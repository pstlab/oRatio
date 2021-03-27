#pragma once

#include "heuristic.h"

namespace ratio
{
  class h_1 : public heuristic
  {
  public:
    h_1(solver &slv);
    h_1(const h_1 &that) = delete;
    ~h_1();

  private:
    void enqueue(flaw &f);
    void propagate_costs(flaw &f);
    void check();

    void build();     // builds the planning graph..
    void add_layer(); // adds a layer to the current planning graph..

#ifdef DEFERRABLE_FLAWS
    bool is_deferrable(flaw &f); // checks whether the given flaw is deferrable..
#endif

  private:
    smt::lit gamma;                              // this literal represents the validity of the current graph..
    std::deque<flaw *> flaw_q;                   // the flaw queue (for the graph building procedure)..
    std::unordered_set<flaw *> visited;          // the visited flaws, for graph cost propagation (and deferrable flaws check)..
    std::unordered_set<smt::var> already_closed; // the already closed flaws (for avoiding duplication of graph pruning constraints)..
  };
} // namespace ratio
