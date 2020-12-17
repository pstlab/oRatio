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
    executor::executor(solver &slv, const size_t &tick_dur, const rational &units_per_tick) : solver_listener(slv), tick_duration(tick_dur), units_per_tick(units_per_tick) {}
    executor::~executor() {}

    std::thread executor::start()
    {
        LOG("current time: " << current_time);
        reset_timelines();
        mtx.lock();
        executing = true;
        mtx.unlock();
        std::chrono::high_resolution_clock::time_point tick_time = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(tick_duration);
        std::thread t([this, &tick_time]() {
            while (true)
            {
                if (!executing)
                    return;
                std::this_thread::sleep_until(tick_time);
                if (!executing)
                    return;
                tick_time += std::chrono::milliseconds(tick_duration);
                mtx.lock();
                tick();
                mtx.unlock();
            }
        });
        return t;
    }

    void executor::stop()
    {
        mtx.lock();
        executing = false;
        mtx.unlock();
    }

    void executor::tick()
    {
        current_time += units_per_tick;
        LOG("current time: " << current_time);

        // we notify that a tick as arised..
        for (const auto &l : listeners)
            l->tick();

        if (!pulses.empty())
            LOG("next thing to do at: " << to_string(*pulses.begin()));

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
                            for (const auto &expr : atm->get_exprs())
                                if (expr.first.compare(END))
                                    if (arith_item *ai = dynamic_cast<arith_item *>(&*expr.second))
                                        frozen_nums.emplace(ai, slv.arith_value(expr.second));
                                    else if (var_item *vi = dynamic_cast<var_item *>(&*expr.second))
                                        frozen_vals.emplace(vi, slv.get_ov_theory().allows(vi->ev, **slv.enum_value(expr.second).begin()));
                        }
                        if (int_pred.is_assignable_from(atm->get_type()))
                        { // we have an impulsive atom..
                            for (const auto &expr : atm->get_exprs())
                                if (expr.first.compare(AT))
                                    if (arith_item *ai = dynamic_cast<arith_item *>(&*expr.second))
                                        frozen_nums.emplace(ai, slv.arith_value(expr.second));
                                    else if (var_item *vi = dynamic_cast<var_item *>(&*expr.second))
                                        frozen_vals.emplace(vi, slv.get_ov_theory().allows(vi->ev, **slv.enum_value(expr.second).begin()));
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
                    slv.pop();

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

    void executor::dont_start_yet(const std::set<atom *> &atoms) { not_starting.insert(atoms.begin(), atoms.end()); }
    void executor::dont_end_yet(const std::set<atom *> &atoms) { not_ending.insert(atoms.begin(), atoms.end()); }
    void executor::failure(const std::set<atom *> &atoms) {}

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
        mtx.lock();
        s_atms.clear();
        e_atms.clear();
        pulses.clear();

        // we collect all the (active) atoms..
        std::unordered_set<atom *> all_atoms;
        for (const auto &p : slv.get_predicates())
            for (const auto &atm : p.second->get_instances())
                if (slv.get_sat_core().value(static_cast<atom &>(*atm).get_sigma()) == True)
                    all_atoms.insert(static_cast<atom *>(&*atm));
        std::queue<type *> q;
        for (const auto &t : slv.get_types())
            if (!t.second->is_primitive())
                q.push(t.second);
        while (!q.empty())
        {
            for (const auto &p : q.front()->get_predicates())
                for (const auto &atm : p.second->get_instances())
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
        mtx.unlock();
    }
} // namespace ratio
