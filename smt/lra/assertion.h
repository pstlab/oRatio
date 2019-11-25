#pragma once

#include "lit.h"
#include "lin.h"
#include "inf_rational.h"
#include <vector>

namespace smt
{

class lra_theory;
class row;

enum op
{
  leq,
  geq
};

/**
 * This class is used for representing assertions.
 * The assertion, controlled by the propositional variable 'b', is represented by the expression 'x' <op> 'v'.
 */
class assertion
{
  friend class lra_theory;
  friend class row;

public:
  assertion(lra_theory &th, const op o, const var b, const var x, const inf_rational &v);
  assertion(const assertion &orig) = delete;
  virtual ~assertion();

private:
  bool propagate_lb(const var &x, std::vector<lit> &cnfl); // propagates the lower bound of variable 'x' on the assertion returning whether propagation is successful..
  bool propagate_ub(const var &x, std::vector<lit> &cnfl); // propagates the upper bound of variable 'x' on the assertion returning whether propagation is successful

public:
  std::string to_string() const;

private:
  lra_theory &th;
  const op o;           // the kind of operator..
  const var b;          // the propositional variable associated to the assertion..
  const var x;          // the numeric variable..
  const inf_rational v; // the constant..
};
} // namespace smt