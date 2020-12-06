#include "executor.h"
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

    executor::executor(core &cr) : core_listener(cr) {}
    executor::~executor() {}

    void executor::set_interval(const size_t &interval)
    {
        executing = true;
        std::chrono::high_resolution_clock::time_point start;
        std::chrono::high_resolution_clock::time_point end;
        std::thread t([this, &start, &end, &interval]() {
            predicate &int_pred = cr.get_predicate("Interval");
            while (true)
            {
                if (!executing)
                    return;
                std::this_thread::sleep_for(std::chrono::milliseconds(interval) - (end - start));
                if (!executing)
                    return;
                start = std::chrono::high_resolution_clock::now();

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

                for (const auto &atm : all_atoms)
                {
                    if (int_pred.is_assignable_from(atm->get_type()))
                    {
                        arith_expr s_expr = atm->get(START);
                        arith_expr e_expr = atm->get(END);
                        inf_rational start = cr.arith_value(s_expr);
                        inf_rational end = cr.arith_value(e_expr);
                        starting_atoms[start].insert(atm);
                        ending_atoms[end].insert(atm);
                        pulses.insert(start);
                        pulses.insert(end);
                    }
                    else
                    {
                        arith_expr at_expr = atm->get(AT);
                        inf_rational at = cr.arith_value(at_expr);
                        starting_atoms[at].insert(atm);
                        ending_atoms[at].insert(atm);
                        pulses.insert(at);
                    }
                }

                // todo: do something..
                end = std::chrono::high_resolution_clock::now();
            }
        });
        t.detach();
    }

    void executor::set_timeout(const size_t &delay)
    {
        executing = true;
        std::thread t([this, delay]() {
            if (!executing)
                return;
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            if (!executing)
                return;
            // todo: do something..
        });
        t.detach();
    }

    void executor::stop() { executing = false; }

    void executor::state_changed()
    {
        cr.get_types();
    }
} // namespace ratio
