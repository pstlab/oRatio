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

public:
  flaw(solver &slv, const std::vector<resolver *> &causes, const bool &exclusive = false, const bool &structural = false);
  flaw(const flaw &orig) = delete;
  ~flaw();

  smt::var get_phi() const { return phi; }
  std::vector<resolver *> get_resolvers() const { return resolvers; }
  std::vector<resolver *> get_causes() const { return causes; }
  std::vector<resolver *> get_supports() const { return supports; }

  smt::rational get_estimated_cost() const;
  resolver *get_best_resolver() const;

private:
  void init();
  void expand();
  virtual void compute_resolvers() = 0;

protected:
  void add_resolver(resolver &r);

protected:
  solver &slv; // the solver this flaw belongs to..

private:
  smt::var phi;                      // the propositional variable indicating whether the flaw is active or not..
  std::vector<resolver *> resolvers; // the resolvers for this flaw..
  std::vector<resolver *> causes;    // the causes for having this flaw..
  std::vector<resolver *> supports;  // the resolvers supported by this flaw..
  const bool exclusive;              // a boolean indicating whether the flaw is exclusive (i.e. exactly one of its resolver can be applied)..
  const bool structural;             // a boolean indicating whether the flaw is structural (i.e. it is not an inconsistency raised from types' consistency checking procedures)..
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
  smt::rational get_estimated_cost() const { return est_cost + intrinsic_cost; }
  flaw &get_effect() const { return effect; }
  std::vector<flaw *> get_preconditions() const { return preconditions; }

private:
  virtual void apply() = 0;

protected:
  solver &slv; // the solver this resolver belongs to..

private:
  const smt::var rho;                                        // the propositional variable indicating whether the resolver is active or not..
  const smt::rational intrinsic_cost;                        // the intrinsic cost of the resolver..
  smt::rational est_cost = smt::rational::POSITIVE_INFINITY; // the estimated cost of the resolver, computed by the heuristic, except for the intrinsic cost..
  flaw &effect;                                              // the flaw solved by this resolver..
  std::vector<flaw *> preconditions;                         // the preconditions of this resolver..
};
} // namespace ratio
