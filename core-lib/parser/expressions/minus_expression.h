#pragma once

#include "expression.h"

namespace ratio
{

namespace ast
{

class minus_expression : public expression
{
public:
  minus_expression(const expression *const e);
  minus_expression(const minus_expression &orig) = delete;
  virtual ~minus_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const expression *const xpr;
};
}
}