#pragma once

#include "expression.h"

namespace ratio
{

namespace ast
{

class lt_expression : public expression
{
public:
  lt_expression(const expression *const l, const expression *const r);
  lt_expression(const lt_expression &orig) = delete;
  virtual ~lt_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const expression *const left;
  const expression *const right;
};
}
}