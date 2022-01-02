#include "graph.h"
#include "resolver.h"
#include "flaw.h"
#include <algorithm>
#include <cassert>

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

    void graph::check()
    {
        assert(slv.root_level());
        switch (slv.get_sat_core().value(gamma))
        {
        case False: // we create a new graph var..
            init();
            [[fallthrough]];
        case Undefined:
            if (!get_flaws().empty())
                if (std::any_of(get_flaws().cbegin(), get_flaws().cend(), [](flaw *f)
                                { return is_positive_infinite(f->get_estimated_cost()); })) // we build/extend the graph..
                    build();
                else // we add a layer to the current graph..
                    add_layer();
#ifdef GRAPH_PRUNING
            prune(); // we prune the graph..
#endif
            do
            {
                slv.take_decision(lit(gamma));
                if (slv.get_sat_core().value(gamma) == False)
                { // the graph has been invalidated..
                    LOG("search has exhausted the graph..");
                    init(); // we create a new graph var..
                    if (!get_flaws().empty())
                        if (std::any_of(get_flaws().cbegin(), get_flaws().cend(), [](flaw *f)
                                        { return is_positive_infinite(f->get_estimated_cost()); })) // we build/extend the graph..
                            build();
                        else // we add a layer to the current graph..
                            add_layer();
#ifdef GRAPH_PRUNING
                    prune(); // we prune the graph..
#endif
                }
            } while (slv.get_sat_core().value(gamma) != True);
        }
    }
} // namespace ratio
