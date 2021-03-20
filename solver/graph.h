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

#ifdef BUILD_LISTENERS
#define G_FIRE_NEW_FLAW(f) slv.fire_new_flaw(f)
#define G_FIRE_FLAW_STATE_CHANGED(f) slv.fire_flaw_state_changed(f)
#define G_FIRE_FLAW_COST_CHANGED(f) slv.fire_flaw_cost_changed(f)
#define G_FIRE_CURRENT_FLAW(f) slv.fire_current_flaw(f)
#define G_FIRE_NEW_RESOLVER(r) slv.fire_new_resolver(r)
#define G_FIRE_RESOLVER_STATE_CHANGED(r) slv.fire_resolver_state_changed(r)
#define G_FIRE_CURRENT_RESOLVER(r) slv.fire_current_resolver(r)
#define G_FIRE_CAUSAL_LINK_ADDED(f, r) slv.fire_causal_link_added(f, r)
#else
#define G_FIRE_NEW_FLAW(f)
#define G_FIRE_FLAW_STATE_CHANGED(f)
#define G_FIRE_FLAW_COST_CHANGED(f)
#define G_FIRE_CURRENT_FLAW(f)
#define G_FIRE_NEW_RESOLVER(r)
#define G_FIRE_RESOLVER_STATE_CHANGED(r)
#define G_FIRE_CURRENT_RESOLVER(r)
#define G_FIRE_CAUSAL_LINK_ADDED(f, r)
#endif

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

    solver &get_solver() const noexcept { return slv; }

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
