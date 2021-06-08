#pragma once

#include "solver.h"

namespace ratio
{
  class flaw;

  class heuristic
  {
    friend class solver;

  public:
    heuristic(solver &slv);
    heuristic(const heuristic &that) = delete;
    virtual ~heuristic();

    inline solver &get_solver() const noexcept { return slv; }

  private:
    virtual void enqueue(flaw &f) = 0;
    virtual void propagate_costs(flaw &f) = 0;
    virtual void check() = 0; // checks the state of the heuristic..

  protected:
    inline void new_flaw(flaw &f, const bool &enqueue = true) { slv.new_flaw(f, enqueue); }
    inline void expand_flaw(flaw &f) { slv.expand_flaw(f); }
    inline void set_cost(flaw &f, const smt::rational &cost) { slv.set_cost(f, cost); }

    inline std::unordered_set<flaw *> &get_flaws() { return slv.flaws; }
    inline std::vector<flaw *> flush_pending_flaws() { return slv.flush_pending_flaws(); }
    inline std::vector<std::vector<std::pair<smt::lit, double>>> get_incs() { return slv.get_incs(); }

  protected:
    solver &slv; // the solver this heuristic belongs to..
  };
} // namespace ratio