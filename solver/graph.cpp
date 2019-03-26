#include "graph.h"
#include "solver.h"

namespace ratio
{
graph::graph(solver &slv) : slv(slv), gamma(slv.get_sat_core().new_var()) {}
graph::~graph() {}

flaw::flaw(graph &gr) : gr(gr) {}
flaw::~flaw() {}

resolver::resolver(graph &gr) : gr(gr) {}
resolver::~resolver() {}
} // namespace ratio
