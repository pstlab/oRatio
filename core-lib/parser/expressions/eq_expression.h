#pragma once

#include "expression.h"

namespace ratio
{

namespace ast
{

class eq_expression : public expression
{
public:
  eq_expression(const expression *const l, const expression *const r);
  eq_expression(const eq_expression &orig) = delete;
  virtual ~eq_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const expression *const left;
  const expression *const right;
};
}
}