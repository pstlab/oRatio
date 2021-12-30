#pragma once

#include "solver.h"

#ifdef BUILD_LISTENERS
#define FIRE_CURRENT_FLAW(f) fire_current_flaw(f)
#define FIRE_CURRENT_RESOLVER(r) fire_current_resolver(r)
#else
#define FIRE_CURRENT_FLAW(f)
#define FIRE_CURRENT_RESOLVER(r)
#endif

namespace ratio
{
  class flaw;

  class graph
  {
    friend class solver;

  public:
    graph(solver &slv);
    graph(const graph &that) = delete;
    virtual ~graph() = default;

    inline solver &get_solver() const noexcept { return slv; }

    virtual smt::rational get_estimated_cost(const resolver &r) const noexcept = 0;

  private:
    virtual void init() = 0;
    void check();

    virtual void enqueue(flaw &f) = 0;
    virtual void propagate_costs(flaw &f) = 0;
    virtual void build() = 0; // builds the graph..
#ifdef GRAPH_PRUNING
    virtual void prune() = 0; // prunes the current graph..
#else
    constexpr bool prune()
    {
      return true;
    }
#endif
    virtual void add_layer() = 0; // adds a layer to the graph..

    virtual void push() {}
    virtual void pop() {}
    virtual void activated_flaw(flaw &f);
    virtual void negated_flaw(flaw &f);
    virtual void activated_resolver(resolver &r);
    virtual void negated_resolver(resolver &r);

  protected:
    inline void new_flaw(flaw &f, const bool &enqueue = true) { slv.new_flaw(f, enqueue); }
    inline void expand_flaw(flaw &f)
    {
      // we expand the flaw..
      slv.expand_flaw(f);

      // we propagate the costs starting from the just expanded flaw..
      propagate_costs(f);
    }
    inline void set_cost(flaw &f, const smt::rational &cost) { slv.set_cost(f, cost); }

    inline std::unordered_set<flaw *> &get_flaws() { return slv.flaws; }
    inline std::unordered_set<flaw *> flush_pending_flaws() { return slv.flush_pending_flaws(); }
    inline std::vector<std::vector<std::pair<smt::lit, double>>> get_incs() { return slv.get_incs(); }

#ifdef BUILD_LISTENERS
    inline void fire_current_flaw(const flaw &f) const
    {
      slv.fire_current_flaw(f);
    }
    inline void fire_current_resolver(const resolver &r) const
    {
      slv.fire_current_resolver(r);
    }
#endif

  protected:
    solver &slv;    // the solver this heuristic belongs to..
    smt::var gamma; // this variable represents the validity of the current graph..
  private:
    std::unordered_map<smt::var, std::vector<flaw *>> phis;     // the phi variables (propositional variable to flaws) of the flaws..
    std::unordered_map<smt::var, std::vector<resolver *>> rhos; // the rho variables (propositional variable to resolver) of the resolvers..
  };
} // namespace ratio