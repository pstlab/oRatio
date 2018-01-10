#pragma once

#include "expression.h"

namespace ratio
{

namespace ast
{

class not_expression : public expression
{
public:
  not_expression(const expression *const e);
  not_expression(const not_expression &orig) = delete;
  virtual ~not_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const expression *const xpr;
};
}
}