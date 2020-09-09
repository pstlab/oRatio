#include "solver.h"
#include "graph.h"
#include "bool_flaw.h"
#include "var_flaw.h"
#include "atom_flaw.h"
#include "disjunction_flaw.h"
#include "smart_type.h"
#include "lexer.h"
#include "atom.h"
#include "state_variable.h"
#include "reusable_resource.h"
#ifdef BUILD_GUI
#include "solver_listener.h"
#endif
#include <algorithm>
#include <math.h>
#include <cassert>

using namespace smt;

namespace ratio
{

    solver::solver() : core(), theory(get_sat_core()), gr(*this) {}
    solver::~solver() {}

    void solver::init()
    {
        read(std::vector<std::string>({"init.rddl"}));
        new_types({new state_variable(*this),
                   new reusable_resource(*this)});
    }

    void solver::solve()
    {
        // some cleanings..
        sts.clear();
        std::queue<type *> q;
        for (const auto &t : get_types())
            if (!t.second->is_primitive())
                q.push(t.second);
        while (!q.empty())
        {
            if (smart_type *st = dynamic_cast<smart_type *>(q.front()))
                sts.push_back(st);
            for (const auto &st : q.front()->get_types())
                q.push(st.second);
            q.pop();
        }

        // we set the gamma variable..
        gr.check_gamma();

#ifdef BUILD_GUI
        fire_state_changed();
#endif

#ifdef CHECK_INCONSISTENCIES
        // we solve all the current inconsistencies..
        solve_inconsistencies();

        while (!flaws.empty())
        {
            assert(std::all_of(flaws.begin(), flaws.end(), [this](flaw *const f) { return sat.value(f->phi) == True; }));                                                                                         // all the current flaws must be active..
            assert(std::all_of(flaws.begin(), flaws.end(), [this](flaw *const f) { return std::none_of(f->resolvers.begin(), f->resolvers.end(), [this](resolver *r) { return sat.value(r->rho) == True; }); })); // none of the current flaws must have already been solved..

            // this is the next flaw (i.e. the most expensive one) to be solved..
            auto f_next = std::min_element(flaws.begin(), flaws.end(), [](flaw *const f0, flaw *const f1) { return f0->get_estimated_cost() > f1->get_estimated_cost(); });
            assert(f_next != flaws.end());

#ifdef BUILD_GUI
            fire_current_flaw(**f_next);
#endif
            if (is_infinite((*f_next)->get_estimated_cost()))
            { // we don't know how to solve this flaw: we search..
                next();
                while (std::any_of(flaws.begin(), flaws.end(), [this](flaw *const f) { return is_infinite(f->get_estimated_cost()); }))
                    next();
                // we solve all the current inconsistencies..
                solve_inconsistencies();
                continue;
            }

            // this is the next resolver (i.e. the cheapest one) to be applied..
            auto *res = (*f_next)->get_best_resolver();
#ifdef BUILD_GUI
            fire_current_resolver(*res);
#endif
            assert(!is_infinite(res->get_estimated_cost()));

            // we apply the resolver..
            take_decision(res->get_rho());

            // we solve all the current inconsistencies..
            solve_inconsistencies();
        }
#else
        do
        {
            while (!flaws.empty())
            {
                assert(std::all_of(flaws.begin(), flaws.end(), [this](flaw *const f) { return sat.value(f->phi) == True; }));                                                                                         // all the current flaws must be active..
                assert(std::all_of(flaws.begin(), flaws.end(), [this](flaw *const f) { return std::none_of(f->resolvers.begin(), f->resolvers.end(), [this](resolver *r) { return sat.value(r->rho) == True; }); })); // none of the current flaws must have already been solved..

                // this is the next flaw (i.e. the most expensive one) to be solved..
                auto f_next = std::min_element(flaws.begin(), flaws.end(), [](flaw *const f0, flaw *const f1) { return f0->get_estimated_cost() > f1->get_estimated_cost(); });
                assert(f_next != flaws.end());

#ifdef BUILD_GUI
                fire_current_flaw(**f_next);
#endif
                if (is_infinite((*f_next)->get_estimated_cost()))
                { // we don't know how to solve this flaw: we search..
                    next();
                    while (std::any_of(flaws.begin(), flaws.end(), [this](flaw *const f) { return is_infinite(f->get_estimated_cost()); }))
                        next(); // and we we search..
                    continue;
                }

                // this is the next resolver (i.e. the cheapest one) to be applied..
                auto *res = (*f_next)->get_best_resolver();
#ifdef BUILD_GUI
                fire_current_resolver(*res);
#endif
                assert(!is_infinite(res->get_estimated_cost()));

                // we apply the resolver..
                take_decision(res->get_rho());
            }

            // we solve all the current inconsistencies..
            solve_inconsistencies();
        } while (!flaws.empty());
#endif

        // Hurray!! we have found a solution..
        LOG(std::to_string(trail.size()) << " (" << std::to_string(flaws.size()) << ")");
#ifdef BUILD_GUI
        fire_state_changed();
#endif
    }

#ifdef BUILD_GUI
    void solver::fire_new_flaw(const flaw &f) const
    {
        for (const auto &l : listeners)
            l->new_flaw(f);
    }
    void solver::fire_flaw_state_changed(const flaw &f) const
    {
        for (const auto &l : listeners)
            l->flaw_state_changed(f);
    }
    void solver::fire_flaw_cost_changed(const flaw &f) const
    {
        for (const auto &l : listeners)
            l->flaw_cost_changed(f);
    }
    void solver::fire_current_flaw(const flaw &f) const
    {
        for (const auto &l : listeners)
            l->current_flaw(f);
    }
    void solver::fire_new_resolver(const resolver &r) const
    {
        for (const auto &l : listeners)
            l->new_resolver(r);
    }
    void solver::fire_resolver_state_changed(const resolver &r) const
    {
        for (const auto &l : listeners)
            l->resolver_state_changed(r);
    }
    void solver::fire_current_resolver(const resolver &r) const
    {
        for (const auto &l : listeners)
            l->current_resolver(r);
    }
    void solver::fire_causal_link_added(const flaw &f, const resolver &r) const
    {
        for (const auto &l : listeners)
            l->causal_link_added(f, r);
    }
#endif
} // namespace ratio