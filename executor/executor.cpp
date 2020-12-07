#include "executor.h"
#include "solver.h"
#include "predicate.h"
#include "atom.h"
#include <thread>
#include <chrono>

#define AT "at"
#define START "start"
#define END "end"

using namespace smt;

namespace ratio
{
    executor::executor(solver &slv, const size_t &tick_dur, const rational &units_for_millis) : slv(slv), int_pred(slv.get_predicate("Interval")), imp_pred(slv.get_predicate("Impulse")), tick_duration(tick_dur), units_for_milliseconds(units_for_millis) {}
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
                        {
                            // todo: we freeze the parameters of the starting atoms..
                            // wait! we do not need to do it now!! we can store the information and apply when backtracking for some reason..
                        }
                    for (const auto &atm : ending_atms->second)
                        if (!not_ending.count(atm))
                        { // we freeze the ending parameter of the ending atoms..
                            if (int_pred.is_assignable_from(atm->get_type()))
                            { // we have an interval atom..
                                arith_expr e_expr = atm->get(END);
                                const auto e_lin = e_expr->l * units_for_milliseconds;
                                pending_facts.push_back(slv.get_lra_theory().new_geq(e_lin, rational(static_cast<I>(current_time))));
                                pending_facts.push_back(slv.get_lra_theory().new_leq(e_lin, rational(static_cast<I>(current_time + tick_duration))));
                            }
                            else if (imp_pred.is_assignable_from(atm->get_type()))
                            { // we have an impulsive atom..
                                arith_expr at_expr = atm->get(AT);
                                const auto at_lin = at_expr->l * units_for_milliseconds;
                                pending_facts.push_back(slv.get_lra_theory().new_geq(at_lin, rational(static_cast<I>(current_time))));
                                pending_facts.push_back(slv.get_lra_theory().new_leq(at_lin, rational(static_cast<I>(current_time + tick_duration))));
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
                            {
                                arith_expr s_expr = atm->get(START);
                                pending_facts.push_back(slv.geq(s_expr, slv.new_real(rational(static_cast<I>(current_time))))->l);
                            }
                            else if (imp_pred.is_assignable_from(atm->get_type()))
                            {
                                arith_expr at_expr = atm->get(AT);
                                pending_facts.push_back(slv.geq(at_expr, slv.new_real(rational(static_cast<I>(current_time))))->l);
                            }

                        for (const auto &atm : not_ending)
                            if (int_pred.is_assignable_from(atm->get_type()))
                            {
                                arith_expr e_expr = atm->get(END);
                                pending_facts.push_back(slv.geq(e_expr, slv.new_real(rational(static_cast<I>(current_time))))->l);
                            }
                            else if (imp_pred.is_assignable_from(atm->get_type()))
                            {
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
