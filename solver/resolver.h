#pragma once

#include "rational.h"
#include <vector>

namespace smt
{
typedef size_t var;
} // namespace smt

namespace ratio
{

class solver;
class graph;
class flaw;

class resolver
{
  friend class solver;
  friend class graph;
  friend class flaw;

public:
  resolver(graph &gr, const smt::rational &cost, flaw &eff);
  resolver(graph &gr, const smt::var &r, const smt::rational &cost, flaw &eff);
  resolver(const resolver &that) = delete;
  ~resolver();

  graph &get_graph() const { return gr; }
  smt::var get_rho() const { return rho; }
  smt::rational get_intrinsic_cost() const { return intrinsic_cost; }
  smt::rational get_estimated_cost() const;
  flaw &get_effect() const { return effect; }
  const std::vector<flaw *> &get_preconditions() const { return preconditions; }

#ifdef BUILD_GUI
  virtual std::string get_label() const = 0;
#endif

private:
  /**
   * Applies this resolver, introducing subgoals and/or constraints.
   * 
   * @pre the solver must be at root-level.
   */
  virtual void apply() = 0;

private:
  graph &gr;                          // the graph this resolver belongs to..
  const smt::var rho;                 // the propositional variable indicating whether the resolver is active or not..
  const smt::rational intrinsic_cost; // the intrinsic cost of the resolver..
  flaw &effect;                       // the flaw solved by this resolver..
  std::vector<flaw *> preconditions;  // the preconditions of this resolver..
};
} // namespace ratio