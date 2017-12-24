#pragma once

#include "expression.h"
#include "rational.h"

namespace ratio
{

namespace ast
{

class int_literal_expression : public expression
{
public:
  int_literal_expression(const smt::I &l);
  int_literal_expression(const int_literal_expression &orig) = delete;
  virtual ~int_literal_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const smt::I literal;
};
}
}