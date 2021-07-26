#pragma once

#include "solver.h"

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
    virtual void enqueue(flaw &f) = 0;
    virtual void propagate_costs(flaw &f) = 0;
    virtual void build() = 0;     // builds the graph..
    virtual bool prune() = 0;     // prunes the current graph..
    virtual void add_layer() = 0; // adds a layer to the graph..

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
    inline std::vector<flaw *> flush_pending_flaws() { return slv.flush_pending_flaws(); }
    inline std::vector<std::vector<std::pair<smt::lit, double>>> get_incs() { return slv.get_incs(); }

  protected:
    solver &slv; // the solver this heuristic belongs to..
  private:
    std::unordered_map<smt::var, std::vector<flaw *>> phis;     // the phi variables (propositional variable to flaws) of the flaws..
    std::unordered_map<smt::var, std::vector<resolver *>> rhos; // the rho variables (propositional variable to resolver) of the resolvers..
  };
} // namespace ratio