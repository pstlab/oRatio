#include "executor.h"
#include "core.h"
#include "predicate.h"
#include "atom.h"
#include "executor_listener.h"
#include <thread>
#include <chrono>

#define AT "at"
#define START "start"
#define END "end"

using namespace smt;

namespace ratio
{

    executor::executor(core &cr, const size_t &tick_dur, const rational &units_for_millis) : cr(cr), tick_duration(tick_dur), units_for_milliseconds(units_for_millis) {}
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
                for (const auto &l : listeners)
                    l->tick();
                if (*pulses.begin() < current_time)
                {
                    if (const auto starting_atms = starting_atoms.find(*pulses.begin()); starting_atms != starting_atoms.end())
                        for (const auto &l : listeners)
                            l->starting_atoms(starting_atms->second);
                    if (const auto ending_atms = ending_atoms.find(*pulses.begin()); ending_atms != ending_atoms.end())
                        for (const auto &l : listeners)
                            l->ending_atoms(ending_atms->second);
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
        starting_atoms.clear();
        ending_atoms.clear();
        pulses.clear();

        // we collect all the (active) atoms..
        std::unordered_set<atom *> all_atoms;
        for (const auto &p : cr.get_predicates())
            for (const auto &atm : p.second->get_instances())
                if (cr.get_sat_core().value(static_cast<atom &>(*atm).get_sigma()) == True)
                    all_atoms.insert(static_cast<atom *>(&*atm));
        std::queue<type *> q;
        for (const auto &t : cr.get_types())
            if (!t.second->is_primitive())
                q.push(t.second);
        while (!q.empty())
        {
            for (const auto &p : q.front()->get_predicates())
                for (const auto &atm : p.second->get_instances())
                    if (cr.get_sat_core().value(static_cast<atom &>(*atm).get_sigma()) == True)
                        all_atoms.insert(static_cast<atom *>(&*atm));
            q.pop();
        }

        predicate &int_pred = cr.get_predicate("Interval");
        predicate &imp_pred = cr.get_predicate("Impulse");
        for (const auto &atm : all_atoms)
        {
            if (int_pred.is_assignable_from(atm->get_type()))
            {
                arith_expr s_expr = atm->get(START);
                arith_expr e_expr = atm->get(END);
                inf_rational start = cr.arith_value(s_expr) / units_for_milliseconds;
                inf_rational end = cr.arith_value(e_expr) / units_for_milliseconds;
                starting_atoms[start].insert(atm);
                ending_atoms[end].insert(atm);
                pulses.insert(start);
                pulses.insert(end);
            }
            else if (imp_pred.is_assignable_from(atm->get_type()))
            {
                arith_expr at_expr = atm->get(AT);
                inf_rational at = cr.arith_value(at_expr) / units_for_milliseconds;
                starting_atoms[at].insert(atm);
                ending_atoms[at].insert(atm);
                pulses.insert(at);
            }
        }
        mtx.unlock();
    }
} // namespace ratio
