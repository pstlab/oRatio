#pragma once

#include "graph.h"

namespace ratio
{
  class h_1 : public graph
  {
  public:
    h_1(solver &slv);
    h_1(const h_1 &that) = delete;
    ~h_1();

  private:
    void enqueue(flaw &f);
    void propagate_costs(flaw &f);

    void build() override;     // builds the h_1 planning graph..
    void prune() override;     // prunes the current h_1 planning graph..
    void add_layer() override; // adds a layer to the current planning graph..

#ifdef DEFERRABLE_FLAWS
    bool is_deferrable(flaw &f); // checks whether the given flaw is deferrable..
#endif

  private:
    std::deque<flaw *> flaw_q;          // the flaw queue (for the graph building procedure)..
    std::unordered_set<flaw *> visited; // the visited flaws, for graph cost propagation (and deferrable flaws check)..
  };
} // namespace ratio
