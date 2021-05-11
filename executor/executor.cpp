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
    EXECUTOR_EXPORT executor::executor(solver &slv, const rational &units_per_tick) : core_listener(slv), solver_listener(slv), units_per_tick(units_per_tick) { reset_timelines(); }
    EXECUTOR_EXPORT executor::~executor() {}

    EXECUTOR_EXPORT void executor::tick()
    {
        current_time += units_per_tick;
        LOG("current time: " << current_time);

        // we notify that a tick has arised..
        for (const auto &l : listeners)
            l->tick(current_time);

        const auto &int_pred = slv.get_predicate("Interval");
        const auto &imp_pred = slv.get_predicate("Impulse");

        if (*pulses.begin() < static_cast<I>(current_time))
        { // we have something to do..
            const auto starting_atms = s_atms.find(*pulses.begin());
            const auto ending_atms = e_atms.find(*pulses.begin());
            if (starting_atms != s_atms.end()) // we notify that we are starting some atoms..
                for (const auto &l : listeners)
                    l->starting(starting_atms->second);
            if (ending_atms != e_atms.end()) // we notify that we are ending some atoms..
                for (const auto &l : listeners)
                    l->ending(ending_atms->second);

            if (starting_atms != s_atms.end()) // some atoms are starting..
                for (const auto &atm : starting_atms->second)
                    if (!not_starting.count(atm))
                    { // we freeze the parameters of the starting atoms..
                        if (int_pred.is_assignable_from(atm->get_type()))
                        { // we have an interval atom..
                            for (const auto &[xpr_name, xpr] : atm->get_exprs())
                                if (xpr_name.compare(END))
                                    if (arith_item *ai = dynamic_cast<arith_item *>(&*xpr))
                                        frozen_nums.emplace(ai, slv.arith_value(xpr));
                                    else if (var_item *vi = dynamic_cast<var_item *>(&*xpr))
                                        frozen_vals.emplace(vi, slv.get_ov_theory().allows(vi->ev, **slv.enum_value(xpr).begin()));
                        }
                        if (int_pred.is_assignable_from(atm->get_type()))
                        { // we have an impulsive atom..
                            for (const auto &[xpr_name, xpr] : atm->get_exprs())
                                if (xpr_name.compare(AT))
                                    if (arith_item *ai = dynamic_cast<arith_item *>(&*xpr))
                                        frozen_nums.emplace(ai, slv.arith_value(xpr));
                                    else if (var_item *vi = dynamic_cast<var_item *>(&*xpr))
                                        frozen_vals.emplace(vi, slv.get_ov_theory().allows(vi->ev, **slv.enum_value(xpr).begin()));
                        }
                    }

            if (ending_atms != e_atms.end()) // some atoms are ending..
                for (const auto &atm : ending_atms->second)
                    if (!not_ending.count(atm))
                    { // we freeze the ending parameter of the ending atoms..
                        if (int_pred.is_assignable_from(atm->get_type()))
                        { // we have an interval atom..
                            arith_expr e_expr = atm->get(END);
                            frozen_nums.emplace(&*e_expr, slv.arith_value(e_expr));
                        }
                        else if (imp_pred.is_assignable_from(atm->get_type()))
                        { // we have an impulsive atom..
                            arith_expr at_expr = atm->get(AT);
                            frozen_nums.emplace(&*at_expr, slv.arith_value(at_expr));
                        }
                    }

            // we make some cleanings..
            if (starting_atms != s_atms.end())
                s_atms.erase(starting_atms);
            if (ending_atms != e_atms.end())
                e_atms.erase(ending_atms);
            pulses.erase(pulses.begin());

            if (!not_starting.empty() || !not_ending.empty())
            { // we have to delay something..
                for (const auto &atm : not_starting)
                    if (int_pred.is_assignable_from(atm->get_type()))
                    { // we have an interval atom..
                        arith_expr s_expr = atm->get(START);
                        pending_facts.push_back(slv.geq(s_expr, slv.new_real(rational(static_cast<I>(current_time))))->l);
                    }
                    else if (imp_pred.is_assignable_from(atm->get_type()))
                    { // we have an impulsive atom..
                        arith_expr at_expr = atm->get(AT);
                        pending_facts.push_back(slv.geq(at_expr, slv.new_real(rational(static_cast<I>(current_time))))->l);
                    }

                for (const auto &atm : not_ending)
                    if (int_pred.is_assignable_from(atm->get_type()))
                    { // we have an interval atom..
                        arith_expr e_expr = atm->get(END);
                        pending_facts.push_back(slv.geq(e_expr, slv.new_real(rational(static_cast<I>(current_time))))->l);
                    }
                    else if (imp_pred.is_assignable_from(atm->get_type()))
                    { // we have an impulsive atom..
                        arith_expr at_expr = atm->get(AT);
                        pending_facts.push_back(slv.geq(at_expr, slv.new_real(rational(static_cast<I>(current_time))))->l);
                    }

                // we backtrack at root-level..
                while (!slv.root_level())
                    slv.get_sat_core().pop();

                // we assert the pending facts..
                slv.assert_facts(pending_facts);
                pending_facts.clear();

                // we solve the problem again..
                slv.solve();

                // we reset the timelines..
                reset_timelines();
            }
        }
    }

    EXECUTOR_EXPORT void executor::dont_start_yet(const std::set<atom *> &atoms) { not_starting.insert(atoms.begin(), atoms.end()); }
    EXECUTOR_EXPORT void executor::dont_end_yet(const std::set<atom *> &atoms) { not_ending.insert(atoms.begin(), atoms.end()); }
    EXECUTOR_EXPORT void executor::failure(const std::set<atom *> &atoms) {}

    void executor::solution_found() { reset_timelines(); }
    void executor::inconsistent_problem()
    {
        s_atms.clear();
        e_atms.clear();
        pulses.clear();
    }

    void executor::resolver_created(const resolver &r)
    { // we cannot plan in the past..
        if (const atom_flaw::activate_goal *ag = dynamic_cast<const atom_flaw::activate_goal *>(&r))
        {
            const auto &int_pred = slv.get_predicate("Interval");
            const auto &imp_pred = slv.get_predicate("Impulse");
            const atom &atm = dynamic_cast<const atom_flaw *>(&ag->get_effect())->get_atom();
            if (int_pred.is_assignable_from(atm.get_type()))
            { // we have an interval atom..
                arith_expr s_expr = atm.get(START);
                slv.get_sat_core().new_clause({!ag->get_rho(), slv.geq(s_expr, slv.new_real(current_time))->l});
            }
            else if (imp_pred.is_assignable_from(atm.get_type()))
            { // we have an impulsive atom..
                arith_expr at_expr = atm.get(AT);
                slv.get_sat_core().new_clause({!ag->get_rho(), slv.geq(at_expr, slv.new_real(current_time))->l});
            }
        }
    }

    void executor::reset_timelines()
    {
        s_atms.clear();
        e_atms.clear();
        pulses.clear();

        // we collect all the (active) atoms..
        std::unordered_set<atom *> all_atoms;
        for (const auto &[pred_name, pred] : slv.get_predicates())
            for (const auto &atm : pred->get_instances())
                if (slv.get_sat_core().value(static_cast<atom &>(*atm).get_sigma()) == True)
                    all_atoms.insert(static_cast<atom *>(&*atm));
        std::queue<type *> q;
        for (const auto &[tp_name, tp] : slv.get_types())
            if (!tp->is_primitive())
                q.push(tp);
        while (!q.empty())
        {
            for (const auto &[pred_name, pred] : q.front()->get_predicates())
                for (const auto &atm : pred->get_instances())
                    if (slv.get_sat_core().value(static_cast<atom &>(*atm).get_sigma()) == True)
                        all_atoms.insert(static_cast<atom *>(&*atm));
            q.pop();
        }

        const auto &int_pred = slv.get_predicate("Interval");
        const auto &imp_pred = slv.get_predicate("Impulse");
        for (const auto &atm : all_atoms)
        {
            if (int_pred.is_assignable_from(atm->get_type()))
            {
                arith_expr s_expr = atm->get(START);
                arith_expr e_expr = atm->get(END);
                inf_rational start = slv.arith_value(s_expr);
                if (start < current_time)
                    continue;
                inf_rational end = slv.arith_value(e_expr);
                s_atms[start].insert(atm);
                e_atms[end].insert(atm);
                pulses.insert(start);
                pulses.insert(end);
            }
            else if (imp_pred.is_assignable_from(atm->get_type()))
            {
                arith_expr at_expr = atm->get(AT);
                inf_rational at = slv.arith_value(at_expr);
                if (at < current_time)
                    continue;
                s_atms[at].insert(atm);
                e_atms[at].insert(atm);
                pulses.insert(at);
            }
        }
    }
} // namespace ratio
