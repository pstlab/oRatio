#include "executor.h"
#include "solver.h"
#include "predicate.h"
#include "atom.h"
#include "atom_flaw.h"
#include <thread>
#include <chrono>

#define AT "at"
#define START "start"
#define END "end"

using namespace smt;

namespace ratio
{
    executor::executor(solver &slv, const size_t &tick_dur, const rational &units_for_millis) : solver_listener(slv), int_pred(slv.get_predicate("Interval")), imp_pred(slv.get_predicate("Impulse")), tick_duration(tick_dur), units_for_milliseconds(units_for_millis) {}
    executor::~executor() {}

    void executor::start()
    {
        mtx.lock();
        executing = true;
        mtx.unlock();
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        std::chrono::high_resolution_clock::time_point end = start;
        std::thread t([this, &start, &end]() {
            while (true)
            {
                if (!executing)
                    return;
                std::this_thread::sleep_for(std::chrono::milliseconds(tick_duration) - (end - start));
                if (!executing)
                    return;
                start = std::chrono::high_resolution_clock::now();
                mtx.lock();
                current_time += tick_duration;

                // we notify that a tick as arised..
                tick();

                if (*pulses.begin() < static_cast<I>(current_time))
                {
                    const auto starting_atms = s_atms.find(*pulses.begin());
                    const auto ending_atms = e_atms.find(*pulses.begin());
                    if (starting_atms != s_atms.end()) // we notify that we are starting some atoms..
                        starting(starting_atms->second);
                    if (ending_atms != e_atms.end()) // we notify that we are ending some atoms..
                        ending(ending_atms->second);

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
                    s_atms.erase(starting_atms);
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

                mtx.unlock();
                end = std::chrono::high_resolution_clock::now();
            }
        });
        t.detach();
    }

    void executor::flaw_created(const flaw &f)
    { // we cannot plan in the past..
        if (const atom_flaw *af = dynamic_cast<const atom_flaw *>(&f))
            if (int_pred.is_assignable_from(af->get_atom().get_type()))
            { // we have an interval atom..
                arith_expr s_expr = af->get_atom().get(START);
                slv.get_sat_core().new_clause({lit(af->get_atom().get_sigma(), false), slv.geq(s_expr, slv.new_real(static_cast<I>(current_time)))->l});
            }
            else if (imp_pred.is_assignable_from(af->get_atom().get_type()))
            { // we have an impulsive atom..
                arith_expr at_expr = af->get_atom().get(AT);
                slv.get_sat_core().new_clause({lit(af->get_atom().get_sigma(), false), slv.geq(at_expr, slv.new_real(static_cast<I>(current_time)))->l});
            }
    }

    void executor::stop()
    {
        mtx.lock();
        executing = false;
        mtx.unlock();
    }

    void executor::set_timeout(const size_t &delay)
    {
        mtx.lock();
        executing = true;
        mtx.unlock();
        std::thread t([this, delay]() {
            if (!executing)
                return;
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            if (!executing)
                return;
            mtx.lock();
            // todo: do something..
            mtx.unlock();
        });
        t.detach();
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

        for (const auto &atm : all_atoms)
        {
            if (int_pred.is_assignable_from(atm->get_type()))
            {
                arith_expr s_expr = atm->get(START);
                arith_expr e_expr = atm->get(END);
                inf_rational start = slv.arith_value(s_expr) / units_for_milliseconds;
                if (start < static_cast<I>(current_time))
                    continue;
                inf_rational end = slv.arith_value(e_expr) / units_for_milliseconds;
                s_atms[start].insert(atm);
                e_atms[end].insert(atm);
                pulses.insert(start);
                pulses.insert(end);
            }
            else if (imp_pred.is_assignable_from(atm->get_type()))
            {
                arith_expr at_expr = atm->get(AT);
                inf_rational at = slv.arith_value(at_expr) / units_for_milliseconds;
                if (at < static_cast<I>(current_time))
                    continue;
                s_atms[at].insert(atm);
                e_atms[at].insert(atm);
                pulses.insert(at);
            }
        }
        mtx.unlock();
    }
} // namespace ratio
