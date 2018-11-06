#pragma once

#include "rational.h"
#include <vector>

namespace smt
{
typedef size_t var;
}

namespace ratio
{

class solver;
class resolver;

class flaw
{
  friend class solver;
  friend class resolver;

public:
  flaw(solver &slv, const std::vector<resolver *> &causes);
  flaw(const flaw &orig) = delete;
  ~flaw();

  smt::var get_phi() const { return phi; }

protected:
  solver &slv; // the solver this flaw belongs to..

private:
  smt::var phi;                      // the propositional variable indicating whether the flaw is active or not..
  std::vector<resolver *> resolvers; // the resolvers for this flaw..
  std::vector<resolver *> causes;    // the causes for having this flaw..
};

class resolver
{
  friend class solver;
  friend class flaw;

public:
  resolver(solver &slv, const smt::rational &cost, flaw &eff);
  resolver(solver &slv, const smt::var &r, const smt::rational &cost, flaw &eff);
  resolver(const resolver &orig) = delete;
  ~resolver();

  smt::var get_rho() const { return rho; }
  smt::rational get_intrinsic_cost() const { return intrinsic_cost; }
  flaw &get_effect() const { return effect; }
  std::vector<flaw *> get_preconditions() const { return preconditions; }

protected:
  solver &slv; // the solver this resolver belongs to..

private:
  const smt::var rho;                 // the propositional variable indicating whether the resolver is active or not..
  const smt::rational intrinsic_cost; // the intrinsic cost of the resolver..
  flaw &effect;                       // the flaw solved by this resolver..
  std::vector<flaw *> preconditions;  // the preconditions of this resolver..
};
} // namespace ratio
