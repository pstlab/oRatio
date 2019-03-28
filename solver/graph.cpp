#include "graph.h"
#include "solver.h"
#include <cassert>

using namespace smt;

namespace ratio
{
graph::graph(solver &slv) : slv(slv), gamma(slv.get_sat_core().new_var()) {}
graph::~graph() {}

void graph::new_flaw(flaw &f)
{
}

void graph::new_resolver(resolver &r)
{
}

void graph::new_causal_link(flaw &f, resolver &r)
{
}

void graph::set_estimated_cost(resolver &r, const rational &cst)
{
}

const smt::rational graph::evaluate(const std::vector<flaw *> &fs)
{
    rational c_cost;
#ifdef H_MAX
    c_cost = rational::NEGATIVE_INFINITY;
    for (const auto &f : fs)
        if (!f->is_expanded())
            return rational::POSITIVE_INFINITY;
        else
        {
            rational c = f->get_estimated_cost();
            if (c > c_cost)
                c_cost = c;
        }
#endif
#ifdef H_ADD
    for (const auto &f : fs)
        return rational::POSITIVE_INFINITY;
    else c_cost += f->get_estimated_cost();
#endif
    return c_cost;
}

void graph::build() {}

void graph::add_layer() {}

void graph::increase_accuracy() {}

void graph::expand_flaw(flaw &f) {}

void graph::apply_resolver(resolver &r) {}

flaw::flaw(graph &gr, const std::vector<resolver *> &causes, const bool &exclusive) : gr(gr), causes(causes), supports(causes), exclusive(exclusive) {}
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

    // we add this flaw to the preconditions of its causes..
    for (const auto &r : causes)
        r->preconditions.push_back(this);

    if (causes.empty())
        // this flaw is necessarily active..
        phi = TRUE_var;
    else
    {
        // we create a new variable..
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
            throw std::runtime_error("the problem is unsolvable");
    }
    else
    {
        // we need to take a decision for solving this flaw..
        std::vector<lit> r_chs;
        for (const auto &r : resolvers)
            r_chs.push_back(r->rho);
        // we link the phi variable to the resolvers' rho variables..
        if (!(exclusive ? gr.get_solver().get_sat_core().exct_one(r_chs, phi) : gr.get_solver().get_sat_core().disj(r_chs, phi)))
            throw std::runtime_error("the problem is unsolvable");
    }
}

void flaw::add_resolver(resolver &r)
{
    resolvers.push_back(&r);
    gr.new_resolver(r);
}

resolver::resolver(graph &gr, const smt::rational &cost, flaw &eff) : resolver(gr, gr.get_solver().get_sat_core().new_var(), cost, eff) {}
resolver::resolver(graph &gr, const smt::var &r, const smt::rational &cost, flaw &eff) : gr(gr), rho(r), intrinsic_cost(cost), effect(eff) {}
resolver::~resolver() {}
} // namespace ratio
