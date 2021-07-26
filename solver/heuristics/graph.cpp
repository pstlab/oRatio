#include "graph.h"
#include "resolver.h"
#include "flaw.h"

using namespace smt;

namespace ratio
{
    graph::graph(solver &slv) : slv(slv) {}

    void graph::activated_flaw(flaw &) {}
    void graph::negated_flaw(flaw &f) { propagate_costs(f); }
    void graph::activated_resolver(resolver &) {}
    void graph::negated_resolver(resolver &r)
    {
        if (slv.get_sat_core().value(r.get_effect().get_phi()) != False) // we update the cost of the resolver's effect..
            propagate_costs(r.get_effect());
    }
} // namespace ratio
