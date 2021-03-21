#pragma once

#include "lit.h"
#include "lin.h"
#include "inf_rational.h"
#include "json.h"
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
   * An assertion, controlled by the literal 'b', represents the expression 'x' <op> 'v'.
   */
  class assertion
  {
    friend class lra_theory;
    friend class row;

  public:
    assertion(lra_theory &th, const op o, const lit b, const var x, const inf_rational &v);
    assertion(const assertion &orig) = delete;
    virtual ~assertion();

  private:
    bool propagate_lb(const var &x) noexcept; // propagates the lower bound of variable 'x' on the assertion returning whether propagation is successful..
    bool propagate_ub(const var &x) noexcept; // propagates the upper bound of variable 'x' on the assertion returning whether propagation is successful..

    json to_json() const noexcept;

  private:
    lra_theory &th;
    const op o;           // the kind of operator..
    const lit b;          // the literal associated to the assertion..
    const var x;          // the numeric variable..
    const inf_rational v; // the constant..
  };

  /**
   * This class is used for representing rows in the sparse tableau.
   * A row represents an equality constraint between a basic variable and a linear expression of non-basic variables.
   */
  class row
  {
    friend class lra_theory;

  public:
    row(lra_theory &th, const var x, lin l);
    row(const row &orig) = delete;
    virtual ~row();

  private:
    bool propagate_lb(const var &x) noexcept; // propagates the lower bound of variable 'x' on the tableau row returning whether propagation is successful..
    bool propagate_ub(const var &x) noexcept; // propagates the upper bound of variable 'x' on the tableau row returning whether propagation is successful..

    json to_json() const noexcept;

  private:
    lra_theory &th;
    const var x; // the basic variable..
    lin l;       // the linear expression which is constrained to be equal to the basic variable 'x'..
  };
} // namespace smt