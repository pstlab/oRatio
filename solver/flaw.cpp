#include "flaw.h"
#include "solver.h"
#include "resolver.h"
#include <cassert>

using namespace smt;

namespace ratio
{

flaw::flaw(graph &gr, const std::vector<resolver *> &causes, const bool &exclusive) : gr(gr), phi(-1), causes(causes), exclusive(exclusive) {}
flaw::flaw(graph &gr, const smt::var &p, const std::vector<resolver *> &causes, const bool &exclusive) : gr(gr), phi(p), causes(causes), exclusive(exclusive) {}
flaw::~flaw() {}

resolver *flaw::get_best_resolver() const
{
    resolver *c_res = nullptr;
    rational c_cost = rational::POSITIVE_INFINITY;
    for (const auto &r : resolvers)
        if (gr.get_solver().get_sat_core().value(r->get_rho()) != False && r->get_estimated_cost() < c_cost)
        {
            c_res = r;
            c_cost = r->get_estimated_cost();
        }
    return c_res;
}

void flaw::init()
{
    assert(!expanded);
    assert(gr.get_solver().get_sat_core().root_level());

    if (phi == -1)
        if (causes.empty()) // this flaw is necessarily active..
            phi = TRUE_var;
        else
        { // we create a new variable..
            std::vector<lit> cs;
            for (const auto &c : causes)
                cs.push_back(c->rho);

            // this flaw is active iff the conjunction of its causes is active..
            phi = gr.get_solver().get_sat_core().new_conj(cs);
        }
}

void flaw::expand()
{
    assert(!expanded);
    assert(gr.get_solver().get_sat_core().root_level());
    expanded = true; // the flaw is now expanded..

    // we compute the resolvers..
    compute_resolvers();

    // we add causal relations between the flaw and its resolvers (i.e., if the flaw is phi exactly one of its resolvers should be in plan)..
    if (resolvers.empty())
    {
        // there is no way for solving this flaw..
        if (!gr.get_solver().get_sat_core().new_clause({lit(phi, false)})) // we force the phi variable at false..
            throw unsolvable_exception();
    }
    else
    {
        // we need to take a decision for solving this flaw..
        std::vector<lit> r_chs;
        for (const auto &r : resolvers)
            r_chs.push_back(r->rho);
        // we link the phi variable to the resolvers' rho variables..
        if (!(exclusive ? gr.get_solver().get_sat_core().exct_one(r_chs, phi) : gr.get_solver().get_sat_core().disj(r_chs, phi)))
            throw unsolvable_exception();
    }
}

void flaw::add_resolver(resolver &r)
{
    resolvers.push_back(&r);
    gr.new_resolver(r);
}

void flaw::make_precondition_of(resolver &r)
{
    r.preconditions.push_back(this);
    supports.push_back(&r);
}
} // namespace ratio