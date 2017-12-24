#pragma once

#include "expression.h"

namespace ratio
{

namespace ast
{

class leq_expression : public expression
{
public:
  leq_expression(const expression *const l, const expression *const r);
  leq_expression(const leq_expression &orig) = delete;
  virtual ~leq_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const expression *const left;
  const expression *const right;
};
}
}