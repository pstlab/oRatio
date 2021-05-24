#include "executor.h"
#include "executor_listener.h"
#include "solver.h"
#include "predicate.h"
#include "atom.h"
#include "atom_flaw.h"
#include <chrono>

#define AT "at"
#define START "start"
#define END "end"

using namespace smt;

namespace ratio
{
    EXECUTOR_EXPORT executor::executor(solver &slv, const rational &units_per_tick) : core_listener(slv), solver_listener(slv), theory(slv.get_sat_core()), int_pred(slv.get_predicate("Interval")), imp_pred(slv.get_predicate("Impulse")), units_per_tick(units_per_tick) { build_timelines(); }
    EXECUTOR_EXPORT executor::~executor() {}

    EXECUTOR_EXPORT void executor::tick()
    {
        LOG("current time: " << current_time);
    manage_tick:
        while (*pulses.cbegin() <= current_time)
        { // we have something to do..
            if (const auto ending_atms = e_atms.find(*pulses.cbegin()); ending_atms != e_atms.cend())
            { // some atoms are ending..
                // we notify that we are ending some atoms..
                for (const auto &l : listeners)
                    l->ending(ending_atms->second);

                std::set<atom *> e_ex_atms;
                for (const auto &atm : ending_atms->second)
                    if (executing.count(atm))
                    { // the ending atoms are still executing, we need to delay them..
                        arith_expr xpr = is_impulse(*atm) ? atm->get(AT) : atm->get(END);
                        const auto lb = slv.arith_value(xpr) + units_per_tick;
                        lbs[atm].emplace(&*xpr, lb);
                        if (!slv.get_lra_theory().set_lb(slv.get_lra_theory().new_var(xpr->l), lb, lit(atm->get_sigma())))
                            throw execution_exception();
                        e_ex_atms.insert(atm);
                    }
                    else
                    {
                        if (is_impulse(*atm)) // we have an impulsive atom..
                            freeze(*atm, {atm->get(AT)});
                        else if (is_interval(*atm)) // we have an interval atom..
                            freeze(*atm, {atm->get(END)});
                    }

                if (!e_ex_atms.empty())
                { // we have some delays: we remove new possible flaws and (re)build the timelines..
                    if (!slv.get_sat_core().propagate())
                        throw execution_exception();

                    // we solve the problem again..
                    slv.search();
                    while (should_set)
                    {
                        set_values();
                        slv.search();
                    }

                    // we (re)build the timelines..
                    build_timelines();
                    goto manage_tick;
                }

                // we make some cleanings..
                e_atms.erase(ending_atms);
            }

            if (const auto starting_atms = s_atms.find(*pulses.cbegin()); starting_atms != s_atms.cend())
            { // some atoms are starting..
                // we notify that we are starting some atoms..
                for (const auto &l : listeners)
                    l->starting(starting_atms->second);
                for (const auto &atm : starting_atms->second)
                {
                    if (is_interval(*atm)) // we have an interval atom..
                        freeze(*atm, atm->get(START));
                    executing.insert(atm);
                }
                // we make some cleanings..
                s_atms.erase(starting_atms);
            }
            pulses.erase(pulses.cbegin());
        }

        current_time += units_per_tick;

        // we notify that a tick has arised..
        for (const auto &l : listeners)
            l->tick(current_time);
    }

    EXECUTOR_EXPORT void executor::freeze(const atom &atm, const expr &xpr)
    {
        if (arith_item *ai = dynamic_cast<arith_item *>(&*xpr))
        {
            const auto val = slv.arith_value(xpr);
            lbs[&atm].emplace(ai, val);
            ubs[&atm].emplace(ai, val);
            if (!slv.get_lra_theory().set(slv.get_lra_theory().new_var(ai->l), val, lit(atm.get_sigma())))
                throw execution_exception();
        }
        else if (var_item *vi = dynamic_cast<var_item *>(&*xpr))
            frozen_vals.emplace(vi, *slv.enum_value(xpr).begin());
    }

    EXECUTOR_EXPORT void executor::done(const std::unordered_set<atom *> &atoms)
    {
        for (const auto &atm : atoms)
            executing.erase(atm);
    }

    EXECUTOR_EXPORT void executor::failure(const std::unordered_set<atom *> &atoms)
    {
        for (const auto &atm : atoms)
            cnfl.push_back(lit(atm->get_sigma(), false));

        // we analyze the theory's conflict, create a no-good from the analysis and backjump..
        analyze_and_backjump();

        // we solve the problem again..
        while (should_set)
        {
            set_values();
            slv.search();
        }

        // we (re)build the timelines..
        build_timelines();
    }

    bool executor::is_impulse(const atom &atm) const noexcept { return imp_pred.is_assignable_from(atm.get_type()); }
    bool executor::is_interval(const atom &atm) const noexcept { return int_pred.is_assignable_from(atm.get_type()); }

    void executor::solution_found() { build_timelines(); }
    void executor::inconsistent_problem()
    {
        s_atms.clear();
        e_atms.clear();
        pulses.clear();
    }

    void executor::flaw_created(const flaw &f)
    { // we cannot plan in the past..
        if (const atom_flaw *af = dynamic_cast<const atom_flaw *>(&f))
        { // we force each atom to start after the current time..
            const atom &atm = af->get_atom();
            if (is_interval(atm))
            { // we have an interval atom..
                arith_expr s_expr = atm.get(START);
                if (!slv.get_sat_core().new_clause({lit(atm.get_sigma(), false), slv.geq(s_expr, slv.new_real(current_time))->l}))
                    throw execution_exception();
            }
            else if (is_impulse(atm))
            { // we have an impulsive atom..
                arith_expr at_expr = atm.get(AT);
                if (!slv.get_sat_core().new_clause({lit(atm.get_sigma(), false), slv.geq(at_expr, slv.new_real(current_time))->l}))
                    throw execution_exception();
            }
        }
    }

    void executor::build_timelines()
    {
        s_atms.clear();
        e_atms.clear();
        pulses.clear();

        // we collect all the (active) atoms..
        for (const auto &[pred_name, pred] : slv.get_predicates())
            if (pred->is_assignable_from(imp_pred) || pred->is_assignable_from(int_pred))
                for (const auto &atm : pred->get_instances())
                    if (slv.get_sat_core().value(static_cast<atom &>(*atm).get_sigma()) == True)
                        all_atoms.emplace(static_cast<atom *>(&*atm)->get_sigma(), static_cast<atom *>(&*atm));
        std::queue<type *> q;
        for (const auto &[tp_name, tp] : slv.get_types())
            if (!tp->is_primitive())
                q.push(tp);
        while (!q.empty())
        {
            for (const auto &[pred_name, pred] : q.front()->get_predicates())
                if (pred->is_assignable_from(imp_pred) || pred->is_assignable_from(int_pred))
                    for (const auto &atm : pred->get_instances())
                        if (slv.get_sat_core().value(static_cast<atom &>(*atm).get_sigma()) == True)
                            all_atoms.emplace(static_cast<atom *>(&*atm)->get_sigma(), static_cast<atom *>(&*atm));
            q.pop();
        }

        const auto &int_pred = slv.get_predicate("Interval");
        const auto &imp_pred = slv.get_predicate("Impulse");
        for (const auto &[sigma, atm] : all_atoms)
            if (is_impulse(*atm))
            {
                arith_expr at_expr = atm->get(AT);
                inf_rational at = slv.arith_value(at_expr);
                if (at < current_time)
                    continue; // this atom is already in the past..
                s_atms[at].insert(atm);
                e_atms[at].insert(atm);
                pulses.insert(at);
            }
            else if (is_interval(*atm))
            {
                arith_expr s_expr = atm->get(START);
                arith_expr e_expr = atm->get(END);
                inf_rational end = slv.arith_value(e_expr);
                if (end < current_time)
                    continue; // this atom is already in the past..
                inf_rational start = slv.arith_value(s_expr);
                if (start >= current_time)
                {
                    s_atms[start].insert(atm);
                    pulses.insert(start);
                }
                e_atms[end].insert(atm);
                pulses.insert(end);
            }
    }

    void executor::set_values()
    {
        should_set = false;
        for (const auto &[itm, val] : frozen_vals)
            if (!slv.get_ov_theory().value(itm->ev).count(val))
                if (slv.root_level())
                    throw execution_exception();
                else
                    slv.get_sat_core().pop();
        for (auto &&[atm, itm_val] : lbs)
            for (auto &&[a_itm, lb] : itm_val)
                if (!slv.get_lra_theory().set_lb(slv.get_lra_theory().new_var(a_itm->l), lb, lit(atm->get_sigma())))
                    throw execution_exception();
        for (auto &&[atm, itm_val] : ubs)
            for (auto &&[a_itm, ub] : itm_val)
                if (!slv.get_lra_theory().set_ub(slv.get_lra_theory().new_var(a_itm->l), ub, lit(atm->get_sigma())))
                    throw execution_exception();
    }
} // namespace ratio
