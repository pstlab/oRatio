#pragma once

#include "expression.h"
#include "rational.h"

namespace ratio
{

namespace ast
{

class real_literal_expression : public expression
{
public:
  real_literal_expression(const smt::rational &l);
  real_literal_expression(const real_literal_expression &orig) = delete;
  virtual ~real_literal_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const smt::rational literal;
};
}
}