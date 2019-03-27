#include "graph.h"
#include "solver.h"

namespace ratio
{
graph::graph(solver &slv) : slv(slv), gamma(slv.get_sat_core().new_var()) {}
graph::~graph() {}

flaw::flaw(graph &gr, const std::vector<resolver *> &causes, const bool &exclusive) : gr(gr), causes(causes), supports(causes), exclusive(exclusive) {}
flaw::~flaw() {}

resolver::resolver(graph &gr, const smt::rational &cost, flaw &eff) : resolver(gr, gr.slv.get_sat_core().new_var(), cost, eff) {}
resolver::resolver(graph &gr, const smt::var &r, const smt::rational &cost, flaw &eff) : gr(gr), rho(r), intrinsic_cost(cost), effect(eff) {}
resolver::~resolver() {}
} // namespace ratio
