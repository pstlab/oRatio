#pragma once

#include "expression.h"

namespace ratio
{

namespace ast
{

class plus_expression : public expression
{
public:
  plus_expression(const expression *const e);
  plus_expression(const plus_expression &orig) = delete;
  virtual ~plus_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const expression *const xpr;
};
}
}