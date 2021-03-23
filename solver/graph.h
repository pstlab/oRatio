#pragma once

#include "lit.h"
#include "rational.h"
#include <deque>
#include <queue>
#include <unordered_map>
#include <map>
#include <set>
#include <unordered_set>
#include <vector>

namespace ratio
{
  class solver;
  class flaw;
  class resolver;
  class refinement_flaw;
  class refinement_resolver;
  class atom_flaw;

  /**
   * This class is used for building the planning graph and generating the solving heuristic.
   */
  class graph
  {
    friend class solver;
    friend class flaw;
    friend class refinement_flaw;
    friend class refinement_resolver;
    friend class atom_flaw;

  public:
    graph(solver &slv);
    graph(const graph &that) = delete;
    ~graph();

    inline solver &get_solver() const noexcept { return slv; }

  private:
    void enqueue(flaw &f);

    void propagate_costs(flaw &f);

    void build();     // builds the planning graph..
    void add_layer(); // adds a layer to the current planning graph..

#ifdef DEFERRABLE_FLAWS
    bool is_deferrable(flaw &f); // checks whether the given flaw is deferrable..
#endif

    void check_gamma(); // checks and possibly resets the value of gamma..

  private:
    solver &slv;                                 // the solver this graph belongs to..
    smt::lit gamma;                              // this literal represents the validity of the current graph..
    std::deque<flaw *> flaw_q;                   // the flaw queue (for the graph building procedure)..
    std::unordered_set<flaw *> visited;          // the visited flaws, for graph cost propagation (and deferrable flaws check)..
    std::unordered_set<smt::var> already_closed; // the already closed flaws (for avoiding duplication of graph pruning constraints)..
  };
} // namespace ratio
