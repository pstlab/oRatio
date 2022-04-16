#include "executor.h"
#include "executor_listener.h"
#include "solver.h"
#include "predicate.h"
#include "atom.h"
#include "atom_flaw.h"
#include <chrono>
#include <sstream>
#include <cassert>

using namespace smt;

namespace ratio
{
    EXECUTOR_EXPORT executor::executor(solver &slv, const rational &units_per_tick) : core_listener(slv), solver_listener(slv), smt::theory(slv.get_sat_core()), units_per_tick(units_per_tick), xi(slv.get_sat_core().new_var())
    {
        bind(variable(xi));
        build_timelines();
    }

    EXECUTOR_EXPORT void executor::tick()
    {
        LOG("current time: " << to_string(current_time));
    manage_tick:
        while (!pulses.empty() && *pulses.cbegin() <= current_time)
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
                        const arith_expr xpr = slv.is_impulse(*atm) ? atm->get(RATIO_AT) : atm->get(RATIO_START);
                        const auto lb = slv.arith_value(xpr) + units_per_tick;
                        adaptations.at(atm).bounds.at(&*xpr).lb = lb;
                        if (!slv.get_lra_theory().set_lb(slv.get_lra_theory().new_var(xpr->l), lb, adaptations.at(atm).sigma_xi))
                        { // setting the lower bound caused a conflict..
                            swap_conflict(slv.get_lra_theory());
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
                        const arith_expr xpr = slv.is_impulse(*atm) ? atm->get(RATIO_AT) : atm->get(RATIO_END);
                        const auto lb = slv.arith_value(xpr) + units_per_tick;
                        adaptations.at(atm).bounds.at(&*xpr).lb = lb;
                        if (!slv.get_lra_theory().set_lb(slv.get_lra_theory().new_var(xpr->l), lb, adaptations.at(atm).sigma_xi))
                        { // setting the lower bound caused a conflict..
                            swap_conflict(slv.get_lra_theory());
                            if (!backtrack_analyze_and_backjump())
                                throw execution_exception();
                        }
                        delays = true;
                        dont_end.erase(at_atm);
                    }

            if (delays)
            { // we have some delays: we propagate and remove new possible flaws..
                if (!slv.get_sat_core().propagate() || !slv.solve())
                    throw execution_exception();
                goto manage_tick;
            }

            if (const auto starting_atms = s_atms.find(*pulses.cbegin()); starting_atms != s_atms.cend())
            { // we freeze the start of the starting atoms..
                for (auto &atm : starting_atms->second)
                {
                    arith_expr start = atm->get(RATIO_START);
                    freeze(*atm, start);
                }
                // we notify that some atoms are starting their execution..
                for (const auto &l : listeners)
                    l->start(starting_atms->second);
            }
            if (const auto ending_atms = e_atms.find(*pulses.cbegin()); ending_atms != e_atms.cend())
            { // we freeze the at and the end of the ending atoms..
                for (auto &atm : ending_atms->second)
                    if (slv.is_impulse(*atm)) // we have an impulsive atom..
                    {
                        arith_expr at = atm->get(RATIO_AT);
                        freeze(*atm, at);
                    }
                    else if (slv.is_interval(*atm)) // we have an interval atom..
                    {
                        arith_expr end = atm->get(RATIO_END);
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
        if (!backtrack_analyze_and_backjump() || !slv.solve())
            throw execution_exception();
    }

    bool executor::propagate(const smt::lit &p) noexcept
    {
        assert(p == xi);
        // we propagate the active bounds..
        for (const auto &adapt : adaptations)
            if (slv.get_sat_core().value(adapt.second.sigma_xi) == True)
                for (const auto &bnds : adapt.second.bounds)
                    if (!slv.get_lra_theory().set_lb(slv.get_lra_theory().new_var(bnds.first->l), bnds.second.lb, adapt.second.sigma_xi) || !slv.get_lra_theory().set_ub(slv.get_lra_theory().new_var(bnds.first->l), bnds.second.ub, adapt.second.sigma_xi))
                    { // setting the lower bound caused a conflict..
                        swap_conflict(slv.get_lra_theory());
                        return false;
                    }
        return true;
    }

    void executor::solution_found()
    {
        switch (slv.get_sat_core().value(xi))
        {
        case False: // the plan can't be executed anymore..
            throw execution_exception();
        case Undefined: // we enforce the xi variable..
            slv.take_decision(xi);
            break;
        }
        switch (slv.get_sat_core().value(xi))
        {
        case False: // the plan can't be executed anymore..
            throw execution_exception();
        case Undefined: // we attempt to solve the problem again..
            slv.solve();
            break;
        }
        build_timelines();
    }
    void executor::inconsistent_problem()
    {
        s_atms.clear();
        e_atms.clear();
        pulses.clear();
    }

    void executor::flaw_created(const flaw &f)
    { // we cannot plan in the past..
        if (const atom_flaw *af = dynamic_cast<const atom_flaw *>(&f))
        { // .. hence we force each atom to start after the current time..
            atom &atm = af->get_atom();
            // either the atom is not active, or the xi variable is false, or the execution bounds must be enforced..
            atom_adaptation adapt = {slv.get_sat_core().new_conj({atm.get_sigma(), xi}), {}};
            all_atoms.emplace(variable(adapt.sigma_xi), &atm);

            if (slv.is_interval(atm))
            { // we have an interval atom..
                arith_expr s_expr = atm.get(RATIO_START);
                if (slv.get_sat_core().value(af->get_atom().get_sigma()) == True && !slv.get_sat_core().new_clause({slv.geq(s_expr, slv.new_real(current_time))->l}))
                    throw execution_exception();
                const auto c_s_bnds = slv.arith_bounds(s_expr);
                atom_adaptation::exec_bounds s_bnds = {c_s_bnds.first, c_s_bnds.second};
                adapt.bounds.emplace(&*s_expr, s_bnds);
                arith_expr e_expr = atm.get(RATIO_END);
                const auto c_e_bnds = slv.arith_bounds(e_expr);
                atom_adaptation::exec_bounds e_bnds = {c_e_bnds.first, c_e_bnds.second};
                adapt.bounds.emplace(&*e_expr, e_bnds);
            }
            else if (slv.is_impulse(atm))
            { // we have an impulsive atom..
                arith_expr at_expr = atm.get(RATIO_AT);
                if (slv.get_sat_core().value(af->get_atom().get_sigma()) == True && !slv.get_sat_core().new_clause({slv.geq(at_expr, slv.new_real(current_time))->l}))
                    throw execution_exception();
                const auto c_bnds = slv.arith_bounds(at_expr);
                atom_adaptation::exec_bounds bnds = {c_bnds.first, c_bnds.second};
                adapt.bounds.emplace(&*at_expr, bnds);
            }
            adaptations.emplace(&atm, std::move(adapt));
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
                { // the atom is active..
                    if (slv.is_impulse(c_atm))
                    {
                        arith_expr at_expr = atm->get(RATIO_AT);
                        inf_rational at = slv.arith_value(at_expr);
                        if (at < current_time)
                            continue; // this atom is already in the past..
                        s_atms[at].insert(&c_atm);
                        e_atms[at].insert(&c_atm);
                        pulses.insert(at);
                    }
                    else if (slv.is_interval(c_atm))
                    {
                        arith_expr s_expr = atm->get(RATIO_START);
                        arith_expr e_expr = atm->get(RATIO_END);
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
                    }
                }
            }
    }

    void executor::freeze(atom &atm, arith_expr &xpr)
    {
        auto val = slv.arith_value(xpr);
        adaptations.at(&atm).bounds.at(&*xpr).lb = val;
        adaptations.at(&atm).bounds.at(&*xpr).ub = val;
        if (!slv.get_lra_theory().set(slv.get_lra_theory().new_var(xpr->l), val, adaptations.at(&atm).sigma_xi))
        { // freezing the arithmetic expression caused a conflict..
            swap_conflict(slv.get_lra_theory());
            if (!backtrack_analyze_and_backjump())
                throw execution_exception();
        }
    }

    void executor::reset_relevant_predicates()
    {
        relevant_predicates.clear();
        for (const auto &[pred_name, pred] : slv.get_predicates())
            if (slv.is_impulse(*pred) || slv.is_interval(*pred))
                relevant_predicates.insert(pred);
        std::queue<type *> q;
        for (const auto &[tp_name, tp] : slv.get_types())
            if (!tp->is_primitive())
                q.push(tp);
        while (!q.empty())
        {
            for (const auto &[pred_name, pred] : q.front()->get_predicates())
                if (slv.is_impulse(*pred) || slv.is_interval(*pred))
                    relevant_predicates.insert(pred);
            q.pop();
        }
    }
} // namespace ratio
