#include "h_2.h"
#include "atom_flaw.h"
#include "resolver.h"
#include <algorithm>
#include <cassert>

using namespace smt;

namespace ratio
{
    h_2::h_2(solver &slv) : graph(slv) {}

    smt::rational h_2::get_estimated_cost(const resolver &r) const noexcept
    {
        if (slv.get_sat_core().value(r.get_rho()) == False)
            return rational::POSITIVE_INFINITY;
        else if (r.get_preconditions().empty())
            return r.get_intrinsic_cost();

        rational est_cost;
#ifdef H2_MAX
        est_cost = rational::NEGATIVE_INFINITY;
        for (const auto &f : r.get_preconditions())
            if (!f->is_expanded())
                return rational::POSITIVE_INFINITY;
            else // we compute the maximum of the flaws' estimated costs..
            {
                rational c = f->get_estimated_cost();
                if (c > est_cost)
                    est_cost = c;
            }
#endif
#ifdef H2_ADD
        for (const auto &f : r.get_preconditions())
            if (!f->is_expanded())
                return rational::POSITIVE_INFINITY;
            else // we compute the sum of the flaws' estimated costs..
                est_cost += f->get_estimated_cost();
#endif
        return est_cost + r.get_intrinsic_cost();
    }

    void h_2::init() noexcept
    { // we create the gamma variable..
        gamma = slv.get_sat_core().new_var();
        LOG("graph var is: γ" << std::to_string(gamma));
        // we clear the already closed flaws so that they can be closed again..
        already_closed.clear();
    }

    void h_2::enqueue(flaw &f) { flaw_q.push_back(&f); }

    void h_2::propagate_costs(flaw &f)
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

    void h_2::build()
    {
        LOG("building the causal graph..");
        assert(slv.get_sat_core().root_level());

        bool ok;
        do
        {
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

            // we check the graph..
            LOG("checking the graph..");
            checking = true;
            std::vector<flaw *> c_flaws(get_flaws().begin(), get_flaws().end());
            // we sort the current flaws according to their estimated costs..
            std::sort(c_flaws.begin(), c_flaws.end(), [](flaw *f0, flaw *f1)
                      { return f0->get_estimated_cost() > f1->get_estimated_cost(); });

            ok = std::all_of(c_flaws.cbegin(), c_flaws.cend(), [this](auto &f)
                             { return check(*f); });
            checking = false;
        } while (!ok);

        // we perform some cleanings..
        if (!slv.get_sat_core().simplify_db())
            throw unsolvable_exception();
    }

#ifdef GRAPH_PRUNING
    void h_2::prune()
    {
        LOG("pruning the graph..");
        for (const auto &f : flaw_q)
            if (already_closed.insert(f).second)
                if (!slv.get_sat_core().new_clause({lit(gamma, false), !f->get_phi()}))
                    throw unsolvable_exception();
        if (!slv.get_sat_core().propagate())
            throw unsolvable_exception();
    }
#endif

    void h_2::add_layer()
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
    bool h_2::is_deferrable(flaw &f)
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

    bool h_2::check(flaw &f)
    {
        if (!f.is_expanded())
            return false; // this flaw will be expanded at next layer addition..
        assert(slv.get_sat_core().value(f.get_phi()) == True);
        assert(std::all_of(f.get_resolvers().begin(), f.get_resolvers().end(), [this](auto &r)
                           { return slv.get_sat_core().value(r->get_rho()) == False || std::none_of(r->get_preconditions().begin(), r->get_preconditions().end(), [this](auto &cf)
                                                                                                    { return slv.get_sat_core().value(cf->get_phi()) == False; }); }));

        current_flaw = &f;
        FIRE_CURRENT_FLAW(f);
        if (!visited.count(&f))
        { // we have not seen this flaw yet..
            if (auto it = std::find_if(f.get_resolvers().begin(), f.get_resolvers().end(), [this](auto &r)
                                       { return slv.get_sat_core().value(r->get_rho()) == True; });
                it != f.get_resolvers().end())
            { // a resolver is already applied for the given flaw..
                // we check its active preconditions..
                std::vector<flaw *> c_precs((*it)->get_preconditions().begin(), (*it)->get_preconditions().end());
                // we sort the current flaws according to their estimated costs..
                std::sort(c_precs.begin(), c_precs.end(), [](flaw *f0, flaw *f1)
                          { return f0->get_estimated_cost() > f1->get_estimated_cost(); });
                //preconditions might include consistency-flaws which, depending also from other resolvers, might not be active
                return std::all_of(c_precs.begin(), c_precs.end(), [this](auto &f)
                                   { return slv.get_sat_core().value(f->get_phi()) == Undefined || check(*f); });
            }
            else
            {
                assert(std::none_of(f.get_resolvers().begin(), f.get_resolvers().end(), [this](auto &r)
                                    { return slv.get_sat_core().value(r->get_rho()) == True; }));
                std::vector<resolver *> c_ress;
                c_ress.reserve(f.get_resolvers().size());
                for (const auto &r : f.get_resolvers())
                    if (slv.get_sat_core().value(r->get_rho()) == Undefined)
                        c_ress.push_back(r);

                // we sort the applicable flaws' resolvers according to their estimated costs..
                std::sort(c_ress.begin(), c_ress.end(), [](resolver *r0, resolver *r1)
                          { return r0->get_estimated_cost() < r1->get_estimated_cost(); });

                size_t c_lvl = slv.decision_level();
                for (const auto &r : c_ress)
                {
                    FIRE_CURRENT_RESOLVER(*r);
                    if (is_infinite(r->get_estimated_cost()))
                        break; // either the resolver cannot be applied or it would not lead to a solution..

                    if (!slv.get_sat_core().assume(r->get_rho()))
                        throw unsolvable_exception();

                    // we check whether the resolver has actually been applied..
                    if (slv.decision_level() > c_lvl)
                    { // yep, the resolver has been applied..
                        assert(slv.get_sat_core().value(r->get_rho()) == True);

                        // we check whether an estimated solution for the given problem is still possible with the current assignments..
                        if (std::none_of(get_flaws().begin(), get_flaws().end(), [](auto &f)
                                         { return is_positive_infinite(f->get_estimated_cost()); })) // we still have an estimated solution, hence we check for the resolver's active preconditions (again, resolver's preconditions might include consistency-flaws which, depending also from other resolvers, might not be active)..
                            if (is_unification(*r) || std::all_of(r->get_preconditions().begin(), r->get_preconditions().end(), [this](auto &f)
                                                                  { return slv.get_sat_core().value(f->get_phi()) == Undefined || check(*f); }))
                            { // either 'r' is a valid unification or all of its preconditions are applicable..
                                slv.get_sat_core().pop();
                                assert(c_lvl == slv.decision_level());
                                return true;
                            }
                        slv.get_sat_core().pop();
                        assert(c_lvl == slv.decision_level());
                    }

                    if (auto c_it = std::find_if(f.get_resolvers().begin(), f.get_resolvers().end(), [this](auto &r)
                                                 { return slv.get_sat_core().value(r->get_rho()) == True; });
                        c_it != f.get_resolvers().end())
                    { // a resolver is already applied for the given flaw..
                        // we check its active preconditions..
                        std::vector<flaw *> c_precs((*c_it)->get_preconditions().begin(), (*c_it)->get_preconditions().end());
                        // we sort the current flaws according to their estimated costs..
                        std::sort(c_precs.begin(), c_precs.end(), [](flaw *f0, flaw *f1)
                                  { return f0->get_estimated_cost() > f1->get_estimated_cost(); });
                        //preconditions might include consistency-flaws which, depending also from other resolvers, might not be active
                        return std::all_of(c_precs.begin(), c_precs.end(), [this](auto &f)
                                           { return slv.get_sat_core().value(f->get_phi()) == Undefined || check(*f); });
                    }
                }
            }
        }
        // it is not possible to estimate a solution for the given flaw..
        return false;
    }

    h_2::refinement::refinement(solver &slv, std::vector<resolver *> causes, std::vector<resolver *> non_mtx_rs) : flaw(slv, std::move(causes)), non_mtx_rs(std::move(non_mtx_rs)) {}
} // namespace ratio
