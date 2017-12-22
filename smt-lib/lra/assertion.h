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

class assertion
{
  friend class lra_theory;
  friend class row;

public:
  assertion(lra_theory &th, const op o, const var b, const var x, const inf_rational &v);
  assertion(const assertion &orig) = delete;
  virtual ~assertion();

private:
  bool propagate_lb(const var &x, std::vector<lit> &cnfl);
  bool propagate_ub(const var &x, std::vector<lit> &cnfl);

private:
  lra_theory &th;
  const op o;
  const var b;
  const var x;
  const inf_rational v;
};
}