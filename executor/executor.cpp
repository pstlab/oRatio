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
    predicate &get_predicate(const solver &slv, const std::string &pred)
    {
        std::vector<std::string> ids;
        size_t start = 0, end = 0;
        do
        {
            end = pred.find('.', start);
            if (end == std::string::npos)
                end = pred.length();
            std::string token = pred.substr(start, end - start);
            if (!token.empty())
                ids.push_back(token);
            start = end + 1;
        } while (end < pred.length() && start < pred.length());

        if (ids.size() == 1)
            return slv.get_predicate(ids[0]);
        else
        {
            type *tp = &slv.get_type(ids[0]);
            for (size_t i = 1; i < ids.size(); ++i)
                if (i == ids.size() - 1)
                    return tp->get_predicate(ids[i]);
                else
                    tp = &tp->get_type(ids[i]);
        }
        // not found
        throw std::out_of_range(pred);
    }

    EXECUTOR_EXPORT executor::executor(solver &slv, const std::string &cnfg_str, const rational &units_per_tick) : core_listener(slv), solver_listener(slv), smt::theory(slv.get_sat_core()), cnfg_str(cnfg_str), int_pred(slv.get_predicate("Interval")), imp_pred(slv.get_predicate("Impulse")), units_per_tick(units_per_tick) { build_timelines(); }
    EXECUTOR_EXPORT executor::~executor() {}

    EXECUTOR_EXPORT void executor::tick()
    {
        LOG("current time: " << current_time);
    manage_tick:
        while (*pulses.cbegin() <= current_time)
        { // we have something to do..
            if (const auto starting_atms = s_atms.find(*pulses.cbegin()); starting_atms != s_atms.cend())
                // we notify that some atoms might be starting their execution..
                for (const auto &l : listeners)
                    l->starting(starting_atms->second);
            if (const auto ending_atms = e_atms.find(*pulses.cbegin()); ending_atms != e_atms.cend())
                // we notify that some atoms might be ending their execution..
                for (const auto &l : listeners)
                    l->ending(ending_atms->second);

            bool delays = false;
            if (const auto starting_atms = s_atms.find(*pulses.cbegin()); starting_atms != s_atms.cend())
                for (const auto &atm : starting_atms->second)
                    if (const auto at_atm = dont_start.find(atm); at_atm != dont_start.end())
                    { // this starting atom is not ready to be started..
                        const arith_expr xpr = is_impulse(*atm) ? atm->get(AT) : atm->get(START);
                        const auto lb = slv.arith_value(xpr) + units_per_tick;
                        lbs[atm].emplace(&*xpr, lb);
                        std::vector<lit> c_cnfl = slv.get_lra_theory().set_lb(slv.get_lra_theory().new_var(xpr->l), lb, lit(atm->get_sigma()));
                        if (!c_cnfl.empty())
                        { // setting the lower bound caused a conflict..
                            std::swap(c_cnfl, cnfl);
                            if (!backtrack_analyze_and_backjump())
                                throw execution_exception();
                        }
                        delays = true;
                        dont_start.erase(at_atm);
                    }
            if (const auto ending_atms = e_atms.find(*pulses.cbegin()); ending_atms != e_atms.cend())
                for (const auto &atm : ending_atms->second)
                    if (const auto at_atm = dont_end.find(atm); at_atm != dont_end.end())
                    { // this ending atom is not ready to be ended..
                        const arith_expr xpr = is_impulse(*atm) ? atm->get(AT) : atm->get(END);
                        const auto lb = slv.arith_value(xpr) + units_per_tick;
                        lbs[atm].emplace(&*xpr, lb);
                        std::vector<lit> c_cnfl = slv.get_lra_theory().set_lb(slv.get_lra_theory().new_var(xpr->l), lb, lit(atm->get_sigma()));
                        if (!c_cnfl.empty())
                        { // setting the lower bound caused a conflict..
                            std::swap(c_cnfl, cnfl);
                            if (!backtrack_analyze_and_backjump())
                                throw execution_exception();
                        }
                        delays = true;
                        dont_end.erase(at_atm);
                    }

            if (delays)
            { // we have some delays: we propagate and remove new possible flaws..
                if (!slv.get_sat_core().propagate())
                    throw execution_exception();

                if (slv.root_level()) // something failed..
                    set_values();

                // we solve the problem again..
                slv.solve();
                goto manage_tick;
            }

            if (const auto starting_atms = s_atms.find(*pulses.cbegin()); starting_atms != s_atms.cend())
            { // we freeze the start of the starting atoms..
                for (auto &atm : starting_atms->second)
                {
                    arith_expr start = atm->get(START);
                    freeze(*atm, start);
                }
                // we notify that some atoms are starting their execution..
                for (const auto &l : listeners)
                    l->start(starting_atms->second);
            }
            if (const auto ending_atms = e_atms.find(*pulses.cbegin()); ending_atms != e_atms.cend())
            { // we freeze the at and the end of the ending atoms..
                for (auto &atm : ending_atms->second)
                    if (is_impulse(*atm)) // we have an impulsive atom..
                    {
                        arith_expr at = atm->get(AT);
                        freeze(*atm, at);
                    }
                    else if (is_interval(*atm)) // we have an interval atom..
                    {
                        arith_expr end = atm->get(END);
                        freeze(*atm, end);
                    }
                // we notify that some atoms are ending their execution..
                for (const auto &l : listeners)
                    l->end(ending_atms->second);
            }

            pulses.erase(pulses.cbegin());
        }

        current_time += units_per_tick;

        // we notify that a tick has arised..
        for (const auto &l : listeners)
            l->tick(current_time);
    }

    EXECUTOR_EXPORT void executor::failure(const std::unordered_set<atom *> &atoms)
    {
        for (const auto &atm : atoms)
            cnfl.push_back(lit(atm->get_sigma(), false));
        // we backtrack to a level at which we can analyze the conflict..
        if (!backtrack_analyze_and_backjump())
            throw execution_exception();

        if (slv.root_level()) // something failed..
            set_values();

        // we solve the problem again..
        slv.solve();
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
            switch (slv.get_sat_core().value(af->get_atom().get_sigma()))
            {
            case True:
                if (is_interval(atm))
                { // we have an interval atom..
                    arith_expr s_expr = atm.get(START);
                    if (!slv.get_sat_core().new_clause({slv.geq(s_expr, slv.new_real(current_time))->l}))
                        throw execution_exception();
                }
                else if (is_impulse(atm))
                { // we have an impulsive atom..
                    arith_expr at_expr = atm.get(AT);
                    if (!slv.get_sat_core().new_clause({slv.geq(at_expr, slv.new_real(current_time))->l}))
                        throw execution_exception();
                }
                break;
            case Undefined:
                bind(af->get_atom().get_sigma());
                break;
            }
        }
    }

    void executor::reset_relevant_predicates()
    {
        relevant_predicates.clear();
        const auto cnfg = smt::json::from_json(std::stringstream(cnfg_str));
        if (cnfg->has("relevant-predicates"))
        { // the configuration identifies some relevant predicates..
            const smt::array_val &notify_s = static_cast<smt::array_val &>(*cnfg->get("relevant-predicates"));
            for (size_t i = 0; i < notify_s.size(); ++i)
            {
                const smt::string_val &pred = static_cast<smt::string_val &>(*notify_s.get(i));
                relevant_predicates.insert(&get_predicate(slv, pred.get()));
            }
        }
        else
        { // all executable predicates are relevant..
            for (const auto &[pred_name, pred] : slv.get_predicates())
                if (pred->is_assignable_from(imp_pred) || pred->is_assignable_from(int_pred))
                    relevant_predicates.insert(pred);
            std::queue<type *> q;
            for (const auto &[tp_name, tp] : slv.get_types())
                if (!tp->is_primitive())
                    q.push(tp);
            while (!q.empty())
            {
                for (const auto &[pred_name, pred] : q.front()->get_predicates())
                    if (pred->is_assignable_from(imp_pred) || pred->is_assignable_from(int_pred))
                        relevant_predicates.insert(pred);
                q.pop();
            }
        }
    }

    void executor::build_timelines()
    {
        LOG("building timelines..");
        s_atms.clear();
        e_atms.clear();
        pulses.clear();

        // we collect all the active relevant atoms..
        for (const auto pred : relevant_predicates)
            for (const auto &atm : pred->get_instances())
            {
                auto &c_atm = static_cast<atom &>(*atm);
                if (slv.get_sat_core().value(c_atm.get_sigma()) == True)
                    if (is_impulse(c_atm))
                    {
                        arith_expr at_expr = atm->get(AT);
                        inf_rational at = slv.arith_value(at_expr);
                        if (at < current_time)
                            continue; // this atom is already in the past..
                        s_atms[at].insert(&c_atm);
                        e_atms[at].insert(&c_atm);
                        pulses.insert(at);
                        all_atoms.emplace(c_atm.get_sigma(), &c_atm);
                    }
                    else if (is_interval(c_atm))
                    {
                        arith_expr s_expr = atm->get(START);
                        arith_expr e_expr = atm->get(END);
                        inf_rational end = slv.arith_value(e_expr);
                        if (end < current_time)
                            continue; // this atom is already in the past..
                        inf_rational start = slv.arith_value(s_expr);
                        if (start >= current_time)
                        {
                            s_atms[start].insert(&c_atm);
                            pulses.insert(start);
                        }
                        e_atms[end].insert(&c_atm);
                        pulses.insert(end);
                        all_atoms.emplace(c_atm.get_sigma(), &c_atm);
                    }
            }
    }

    void executor::freeze(atom &atm, arith_expr &xpr)
    {
        auto val = slv.arith_value(xpr);
        lbs[&atm].emplace(&*xpr, val);
        ubs[&atm].emplace(&*xpr, val);
        std::vector<lit> c_cnfl = slv.get_lra_theory().set(slv.get_lra_theory().new_var(xpr->l), val, lit(atm.get_sigma()));
        if (!c_cnfl.empty())
        { // freezing the arithmetic expression caused a conflict..
            std::swap(c_cnfl, cnfl);
            if (!backtrack_analyze_and_backjump())
                throw execution_exception();
        }
    }

    void executor::set_values()
    {
        for (auto &&[atm, itm_val] : lbs)
            for (auto &&[a_itm, lb] : itm_val)
                if (!slv.get_sat_core().new_clause({lit(atm->get_sigma(), false), slv.geq(a_itm, slv.new_real(current_time))->l}))
                    throw execution_exception();
        lbs.clear();
        for (auto &&[atm, itm_val] : ubs)
            for (auto &&[a_itm, ub] : itm_val)
                if (!slv.get_sat_core().new_clause({lit(atm->get_sigma(), false), slv.leq(a_itm, slv.new_real(current_time))->l}))
                    throw execution_exception();
        ubs.clear();
    }
} // namespace ratio
