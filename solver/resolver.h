#pragma once

#include "lit.h"
#include "rational.h"
#include <vector>

namespace ratio
{
  class solver;
  class flaw;

  class resolver
  {
    friend class solver;
    friend class flaw;

  public:
    resolver(solver &slv, const smt::rational &cost, flaw &eff);
    resolver(solver &slv, const smt::lit &r, const smt::rational &cost, flaw &eff);
    resolver(const resolver &that) = delete;
    ~resolver();

    inline solver &get_solver() const noexcept { return slv; }
    inline smt::lit get_rho() const noexcept { return rho; }
    inline smt::rational get_intrinsic_cost() const noexcept { return intrinsic_cost; }
    smt::rational get_estimated_cost() const noexcept;
    inline flaw &get_effect() const noexcept { return effect; }
    inline const std::vector<flaw *> &get_preconditions() const noexcept { return preconditions; }

    virtual std::string get_label() const = 0;

  private:
    /**
   * Applies this resolver, introducing subgoals and/or constraints.
   * 
   * @pre the solver must be at root-level.
   */
    virtual void apply() = 0;

  private:
    solver &slv;                        // the solver this resolver belongs to..
    const smt::lit rho;                 // the propositional literal indicating whether the resolver is active or not..
    const smt::rational intrinsic_cost; // the intrinsic cost of the resolver..
    flaw &effect;                       // the flaw solved by this resolver..
    std::vector<flaw *> preconditions;  // the preconditions of this resolver..
  };
} // namespace ratio