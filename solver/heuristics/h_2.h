#pragma once

#include "graph.h"
#include "flaw.h"

namespace ratio
{
  class h_2 final : public graph
  {
  public:
    h_2(solver &slv);
    h_2(const h_2 &that) = delete;

    smt::rational get_estimated_cost(const resolver &r) const noexcept override;

  private:
    void init() noexcept override;
    void enqueue(flaw &f);
    void propagate_costs(flaw &f);

    void build() override; // builds the h_2 planning graph..
#ifdef GRAPH_PRUNING
    void prune() override; // prunes the current h_2 planning graph..
#endif
    void add_layer() override; // adds a layer to the current planning graph..

    void push() noexcept override
    {
      if (checking)
        visited.insert(current_flaw);
    }
    void pop() noexcept override
    {
      if (checking)
        visited.erase(current_flaw);
    }
    void activated_flaw(flaw &) noexcept override {}
    void negated_flaw(flaw &) noexcept override {}
    void activated_resolver(resolver &) noexcept override {}
    void negated_resolver(resolver &) noexcept override {}

    bool check(flaw &f);

#ifdef DEFERRABLE_FLAWS
    bool is_deferrable(flaw &f); // checks whether the given flaw is deferrable..
#endif

    class refinement final : public flaw
    {
    public:
      refinement(solver &slv, std::vector<resolver *> causes, std::vector<resolver *> non_mtx_rs);

    private:
      std::vector<resolver *> non_mtx_rs;
    };

  private:
    bool checking = false;
    std::deque<flaw *> flaw_q;          // the flaw queue (for the graph building procedure)..
    std::unordered_set<flaw *> visited; // the visited flaws, for graph cost propagation (and deferrable flaws check)..
    flaw *current_flaw = nullptr;
#ifdef GRAPH_PRUNING
    std::unordered_set<flaw *> already_closed; // already closed flaws (for avoiding duplicating graph pruning constraints)..
#endif
  };
} // namespace ratio
