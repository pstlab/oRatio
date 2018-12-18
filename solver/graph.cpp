#include "graph.h"
#include "solver.h"
#include <cassert>

using namespace smt;

namespace ratio
{

flaw::flaw(solver &slv, const std::vector<resolver *> &causes, const bool &exclusive, const bool &structural) : slv(slv), causes(causes), supports(causes), exclusive(exclusive), structural(structural)
{
    for (const auto &r : causes) // we add this flaw to the preconditions of the causes..
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
        if (slv.get_sat_core().value(r->get_rho()) != False && r->get_estimated_cost() < c_cost)
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
        phi = slv.get_sat_core().new_conj(cs);
    }
}

void flaw::expand()
{
    assert(!expanded);

    // we compute the resolvers..
    compute_resolvers();
    expanded = true; // the flaw is now expanded..

    // we add causal relations between the flaw and its resolvers (i.e., if the flaw is phi exactly one of its resolvers should be in plan)..
    if (resolvers.empty())
    {
        // there is no way for solving this flaw..
        if (!slv.get_sat_core().new_clause({lit(phi, false)})) // we force the phi variable at false..
            throw std::runtime_error("the problem is unsolvable");
    }
    else
    {
        // we need to take a decision for solving this flaw..
        std::vector<lit> r_chs;
        for (const auto &r : resolvers)
            r_chs.push_back(r->rho);
        // we link the phi variable to the resolvers' rho variables..
        if (!(exclusive ? slv.get_sat_core().exct_one(r_chs, phi) : slv.get_sat_core().disj(r_chs, phi)))
            throw std::runtime_error("the problem is unsolvable");
    }
}

void flaw::add_resolver(resolver &r)
{
    resolvers.push_back(&r);
    slv.new_resolver(r);
}

resolver::resolver(solver &slv, const smt::rational &cost, flaw &eff) : resolver(slv, slv.get_sat_core().new_var(), cost, eff) {}
resolver::resolver(solver &slv, const smt::var &r, const smt::rational &cost, flaw &eff) : slv(slv), rho(r), intrinsic_cost(cost), effect(eff) {}
resolver::~resolver() {}
} // namespace ratio
