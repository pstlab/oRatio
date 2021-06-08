#include "h_1.h"
#include "flaw.h"
#include "resolver.h"
#include <algorithm>
#include <cassert>

using namespace smt;

namespace ratio
{
    h_1::h_1(solver &slv) : heuristic(slv), gamma(slv.get_sat_core().new_var()) { LOG("γ is " << to_string(gamma)); }
    h_1::~h_1() {}

    void h_1::enqueue(flaw &f) { flaw_q.push_back(&f); }

    void h_1::propagate_costs(flaw &f)
    {
        rational c_cost; // the current cost..
        if (slv.get_sat_core().value(f.get_phi()) == False)
            c_cost = rational::POSITIVE_INFINITY;
        else
        {
            resolver *chpst_res = f.get_cheapest_resolver(); // the flaw's cheapest resolver..
            c_cost = chpst_res ? chpst_res->get_estimated_cost() : rational::POSITIVE_INFINITY;
        }

        if (f.get_estimated_cost() == c_cost)
            return; // nothing to propagate..
        else if (visited.count(&f))
        { // we are propagating costs within a causal cycle..
            c_cost = rational::POSITIVE_INFINITY;
            if (f.get_estimated_cost() == c_cost)
                return; // nothing to propagate..
        }

        // we update the cost of the flaw..
        set_cost(f, c_cost);

        // we (try to) update the estimated costs of the supports' effects and enqueue them for cost propagation..
        visited.insert(&f);
        for (const auto &supp : f.get_supports())
            if (slv.get_sat_core().value(supp->get_rho()) != False)
                propagate_costs(supp->get_effect());
        visited.erase(&f);
    }

    void h_1::check()
    {
        assert(slv.root_level());
        if (slv.get_sat_core().value(gamma) != False) // a unit clause, not gamma, has been deduced or, simply, gamma has never been set..
            build();                                  // the search procedure may have excluded those parts of the graph that could lead to a solution..
        else
        { // the graph has been invalidated..
            LOG("search has exhausted the graph..");
            // we create a new graph var..
            gamma = lit(slv.get_sat_core().new_var());
            LOG("γ is" << to_string(gamma));
            already_closed.clear();
            if (std::any_of(get_flaws().cbegin(), get_flaws().cend(), [](flaw *f)
                            { return is_positive_infinite(f->get_estimated_cost()); }))
                build(); // we can still build upon the current graph..
            else
                add_layer(); // we add a layer to the current graph..
        }

        LOG("pruning the graph..");
        // these flaws have not been expanded, hence, cannot have a solution..
        for (const auto &f : flaw_q)
            if (already_closed.insert(variable(f->get_phi())).second)
                if (!slv.get_sat_core().new_clause({!gamma, !f->get_phi()}))
                    throw unsolvable_exception();

        slv.take_decision(gamma);
    }

    void h_1::build()
    {
        assert(slv.get_sat_core().root_level());
        LOG("building the causal graph..");

        while (std::any_of(get_flaws().cbegin(), get_flaws().cend(), [](flaw *f)
                           { return is_positive_infinite(f->get_estimated_cost()); }))
        {
            if (flaw_q.empty())
                throw unsolvable_exception();
#ifdef DEFERRABLE_FLAWS
            flaw *c_f = flaw_q.front();
            flaw_q.pop_front();
            assert(!c_f->is_expanded());
            if (slv.get_sat_core().value(c_f->get_phi()) != False)
                if (is_deferrable(*c_f)) // we have a deferrable flaw: we can postpone its expansion..
                    flaw_q.push_back(c_f);
                else
                    expand_flaw(*c_f); // we expand the flaw..
#else
            size_t q_size = flaw_q.size();
            for (size_t i = 0; i < q_size; ++i)
            {
                flaw *c_f = flaw_q.front();
                flaw_q.pop_front();
                assert(!c_f->is_expanded());
                if (slv.get_sat_core().value(c_f->get_phi()) != False)
                    expand_flaw(*c_f); // we expand the flaw..
            }
#endif
        }

        // we extract the inconsistencies (and translate them into flaws)..
        std::vector<std::vector<std::pair<lit, double>>> incs = get_incs();
        for (const auto &f : flush_pending_flaws())
            new_flaw(*f, false); // we add the flaws, without enqueuing, to the planning graph..

        // we perform some cleanings..
        slv.get_sat_core().simplify_db();
    }

    void h_1::add_layer()
    {
        assert(slv.get_sat_core().root_level());
        assert(std::none_of(get_flaws().cbegin(), get_flaws().cend(), [](flaw *f)
                            { return is_positive_infinite(f->get_estimated_cost()); }));
        LOG("adding a layer to the causal graph..");

        std::deque<flaw *> f_q(flaw_q);
        while (std::all_of(f_q.cbegin(), f_q.cend(), [](flaw *f)
                           { return is_infinite(f->get_estimated_cost()); }))
        {
            if (flaw_q.empty())
                throw unsolvable_exception();
            size_t q_size = flaw_q.size();
            for (size_t i = 0; i < q_size; ++i)
            {
                flaw *c_f = flaw_q.front();
                flaw_q.pop_front();
                if (slv.get_sat_core().value(c_f->get_phi()) != False)
                    expand_flaw(*c_f); // we expand the flaw..
            }
        }

        // we extract the inconsistencies (and translate them into flaws)..
        std::vector<std::vector<std::pair<lit, double>>> incs = get_incs();
        for (const auto &f : flush_pending_flaws())
            new_flaw(*f, false); // we add the flaws, without enqueuing, to the planning graph..

        // we perform some cleanings..
        slv.get_sat_core().simplify_db();
    }

#ifdef DEFERRABLE_FLAWS
    bool h_1::is_deferrable(flaw &f)
    {
        if (f.get_estimated_cost() < rational::POSITIVE_INFINITY || std::any_of(f.get_resolvers().cbegin(), f.get_resolvers().cend(), [this](resolver *r)
                                                                                { return slv.get_sat_core().value(r->get_rho()) == True; }))
            return true; // we already have a possible solution for this flaw, thus we defer..
        if (slv.get_sat_core().value(f.get_phi()) == True || visited.count(&f))
            return false; // we necessarily have to solve this flaw: it cannot be deferred..
        visited.insert(&f);
        bool def = std::all_of(f.get_supports().cbegin(), f.get_supports().cend(), [this](resolver *r)
                               { return is_deferrable(r->get_effect()); });
        visited.erase(&f);
        return def;
    }
#endif
} // namespace ratio
