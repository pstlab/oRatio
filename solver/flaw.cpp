#include "flaw.h"
#include "solver.h"
#include "resolver.h"
#include <cassert>

using namespace smt;

namespace ratio
{

flaw::flaw(graph &gr, const std::vector<resolver *> &causes, const bool &exclusive) : gr(gr), phi(causes.empty() ? TRUE_var : gr.get_solver().get_sat_core().new_var()), causes(causes), exclusive(exclusive) {}
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
    assert(gr.get_solver().root_level());

    if (!causes.empty())
    { // we link the new variable to the causes of the flaws..
        std::vector<lit> cs;
        cs.reserve(causes.size() + 1);
        cs.push_back(phi);
        for (const auto &c : causes)
            cs.push_back(lit(c->rho, false));

        if (!gr.get_solver().get_sat_core().new_clause(cs))
            throw unsolvable_exception();
    }
}

void flaw::expand()
{
    assert(!expanded);
    assert(gr.get_solver().root_level());
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
        r_chs.reserve(resolvers.size() + 1);
        r_chs.push_back(lit(phi, false));
        for (const auto &r : resolvers)
            r_chs.push_back(r->rho);

        // we link the phi variable to the resolvers' rho variables..
        if (!gr.get_solver().get_sat_core().new_clause(r_chs))
            throw unsolvable_exception();

        if (exclusive)
            for (size_t i = 0; i < resolvers.size(); ++i)
                for (size_t j = i + 1; j < resolvers.size(); ++j)
                    if (!gr.get_solver().get_sat_core().new_clause({lit(resolvers[i]->rho, false), lit(resolvers[j]->rho, false)}))
                        throw unsolvable_exception();
    }
}

void flaw::add_resolver(resolver &r)
{
    // the activation of the resolver activates (and solves!) the flaw..
    if (!gr.get_solver().get_sat_core().new_clause({lit(r.rho, false), phi}))
        throw unsolvable_exception();
    resolvers.push_back(&r);
    gr.new_resolver(r);
}

void flaw::make_precondition_of(resolver &r)
{
    r.preconditions.push_back(this);
    supports.push_back(&r);
}
} // namespace ratio