#include "flaw.h"
#include "solver.h"
#include "resolver.h"
#include <cassert>

using namespace smt;

namespace ratio
{
    flaw::flaw(solver &slv, const std::vector<resolver *> &causes, const bool &exclusive) : slv(slv), position(slv.get_idl_theory().new_var()), causes(causes), exclusive(exclusive) {}
    flaw::~flaw() {}

    resolver *flaw::get_cheapest_resolver() const noexcept
    {
        resolver *c_res = nullptr;
        rational c_cost = rational::POSITIVE_INFINITY;
        for (const auto &r : resolvers)
            if (slv.get_sat_core().value(r->get_rho()) != False && r->get_estimated_cost() < c_cost)
            {
                c_res = r;
                c_cost = r->get_estimated_cost();
            }
        return c_res;
    }

    void flaw::init() noexcept
    {
        assert(!expanded);
        assert(slv.root_level());

        bool add_distance = slv.get_sat_core().new_clause({slv.get_idl_theory().new_distance(position, 0, 0)});
        assert(add_distance);

        std::vector<lit> cs;
        cs.reserve(causes.size());
        for (const auto &c : causes)
        {
            c->preconditions.push_back(this); // this flaw is a precondition of its 'c' cause..
            supports.push_back(c);            // .. and it also supports its 'c' cause..
            cs.push_back(c->rho);
            bool dist = slv.get_sat_core().new_clause({slv.get_idl_theory().new_distance(c->effect.position, position, -1)});
            assert(dist);
        }
        // we initialize the phi variable as the conjunction of the causes' rho variables..
        phi = slv.get_sat_core().new_conj(cs);
    }

    void flaw::expand()
    {
        assert(!expanded);
        assert(slv.root_level());
        expanded = true; // the flaw is now expanded..

        // we compute the resolvers..
        compute_resolvers();

        // we add causal relations between the flaw and its resolvers (i.e., if the flaw is phi exactly one of its resolvers should be in plan)..
        if (resolvers.empty())
        { // there is no way for solving this flaw: we force the phi variable at false..
            if (!slv.get_sat_core().new_clause({!phi}))
                throw unsolvable_exception();
        }
        else
        { // we need to take a decision for solving this flaw..
            std::vector<lit> r_chs;
            r_chs.reserve(resolvers.size() + 1);
            r_chs.push_back(!phi);
            for (const auto &r : resolvers)
                r_chs.push_back(r->rho);

            // we link the phi variable to the resolvers' rho variables..
            if (!slv.get_sat_core().new_clause(r_chs))
                throw unsolvable_exception();

            if (exclusive) // we make the resolvers mutually exclusive..
                for (size_t i = 0; i < resolvers.size(); ++i)
                    for (size_t j = i + 1; j < resolvers.size(); ++j)
                        if (!slv.get_sat_core().new_clause({!resolvers[i]->rho, !resolvers[j]->rho}))
                            throw unsolvable_exception();
        }
    }

    void flaw::add_resolver(resolver &r)
    {
        // the activation of the resolver activates (and solves!) the flaw..
        if (!slv.get_sat_core().new_clause({!r.rho, phi}))
            throw unsolvable_exception();
        resolvers.push_back(&r);
        slv.new_resolver(r);
    }
} // namespace ratio