#include "flaw.h"
#include "resolver.h"
#include "solver.h"
#include <cassert>

using namespace smt;

namespace ratio
{

flaw::flaw(solver &slv, const std::vector<resolver *> &causes, const bool &exclusive, const bool &structural) : slv(slv), exclusive(exclusive), structural(structural), causes(causes), supports(causes)
{
    for (const auto &r : causes)
        r->preconditions.push_back(this);
}

flaw::~flaw() {}

rational flaw::get_estimated_cost() const
{
    resolver *c_res = get_best_resolver();
    if (c_res)
        return c_res->get_estimated_cost();
    else
        return rational::POSITIVE_INFINITY;
}

resolver *flaw::get_best_resolver() const
{
    resolver *c_res = nullptr;
    rational c_cost = rational::POSITIVE_INFINITY;
    for (const auto &r : resolvers)
        if (slv.sat_cr.value(r->get_rho()) != False && r->get_estimated_cost() < c_cost)
        {
            c_res = r;
            c_cost = r->get_estimated_cost();
        }
    return c_res;
}

void flaw::init()
{
    assert(!expanded);

    if (causes.empty())
        // the flaw is necessarily active..
        phi = TRUE_var;
    else
    {
        // we create a new variable..
        std::vector<lit> cs;
        for (const auto &c : causes)
            cs.push_back(c->rho);

        // the flaw is active if the conjunction of its causes is active..
        phi = slv.sat_cr.new_conj(cs);
    }

    switch (slv.sat_cr.value(phi))
    {
    case True: // we have a top-level (a landmark) flaw..
        slv.flaws.insert(this);
        break;
    case Undefined: // we listen for the flaw to become active..
        slv.phis[phi].push_back(this);
        slv.bind(phi);
        break;
    }
}

void flaw::expand()
{
    assert(!expanded);
    assert(slv.sat_cr.value(phi) != False);

    // we compute the resolvers..
    compute_resolvers();
    expanded = true;

    // we add causal relations between the flaw and its resolvers (i.e., if the flaw is phi exactly one of its resolvers should be in plan)..
    if (resolvers.empty())
    {
        // there is no way for solving this flaw..
        if (!slv.sat_cr.new_clause({lit(phi, false)}))
            throw unsolvable_exception();
    }
    else
    {
        // we need to take a decision for solving this flaw..
        std::vector<lit> r_chs;
        for (const auto &r : resolvers)
            r_chs.push_back(r->rho);
        if (!slv.sat_cr.new_clause({lit(phi, false), exclusive ? slv.sat_cr.new_exct_one(r_chs) : slv.sat_cr.new_disj(r_chs)}))
            throw unsolvable_exception();
    }
}

void flaw::add_resolver(resolver &r)
{
    resolvers.push_back(&r);
    slv.new_resolver(r);
}
}