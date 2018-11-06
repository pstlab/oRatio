#include "graph.h"
#include "solver.h"

namespace ratio
{

flaw::flaw(solver &slv, const std::vector<resolver *> &causes) : slv(slv), causes(causes)
{
    for (const auto &r : causes) // we add this flaw to the preconditions of the causes..
        r->preconditions.push_back(this);
}
flaw::~flaw() {}

resolver::resolver(solver &slv, const smt::rational &cost, flaw &eff) : resolver(slv, slv.get_sat_core().new_var(), cost, eff) {}
resolver::resolver(solver &slv, const smt::var &r, const smt::rational &cost, flaw &eff) : slv(slv), rho(r), intrinsic_cost(cost), effect(eff) {}
resolver::~resolver() {}
} // namespace ratio
