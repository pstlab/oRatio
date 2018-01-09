#pragma once

#include "rational.h"
#include <vector>
#include <string>

namespace smt
{
typedef size_t var;
}

namespace ratio
{

class solver;
class flaw;

class resolver
{
  friend class solver;
  friend class flaw;

public:
  resolver(solver &slv, const smt::var &r, const smt::rational &cost, flaw &eff);
  resolver(solver &slv, const smt::rational &cost, flaw &eff);
  resolver(const resolver &orig) = delete;
  virtual ~resolver();

private:
  void init();
  virtual void apply() = 0;

public:
  const smt::var &get_rho() const { return rho; }
  flaw &get_effect() const { return effect; }
  std::vector<flaw *> get_preconditions() const { return preconditions; }
  smt::rational get_intrinsic_cost() const { return intrinsic_cost; }
  smt::rational get_estimated_cost() const { return est_cost + intrinsic_cost; }

  virtual std::string get_label() const = 0;

protected:
  solver &slv;                                               // the solver this resolver belongs to..
  const smt::var rho;                                        // the propositional variable indicating whether the resolver is active or not..
  const smt::rational intrinsic_cost;                        // the intrinsic cost of the resolver..
  std::vector<flaw *> preconditions;                         // the preconditions of this resolver..
  flaw &effect;                                              // the flaw solved by this resolver..
  smt::rational est_cost = smt::rational::POSITIVE_INFINITY; // the estimated cost of the resolver..
};
}