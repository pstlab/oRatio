#pragma once

#include "lit.h"
#include "lin.h"
#include "inf_rational.h"
#include <vector>

namespace smt
{

class lra_theory;

class row
{
  friend class lra_theory;

public:
  row(lra_theory &th, const var x, lin l);
  row(const row &orig) = delete;
  virtual ~row();

private:
  bool propagate_lb(const var &x, std::vector<lit> &cnfl); // propagates the lower bound of variable 'x' on the tableau row returning whether propagation is successful..
  bool propagate_ub(const var &x, std::vector<lit> &cnfl); // propagates the upper bound of variable 'x' on the tableau row returning whether propagation is successful..

public:
  std::string to_string() const;

private:
  lra_theory &th;
  const var x; // the basic variable..
  lin l;       // the linear expression which is constrained to be equal to the basic variable 'x'..
};
} // namespace smt