#pragma once

#include "expression.h"

namespace ratio
{

namespace ast
{

class neq_expression : public expression
{
public:
  neq_expression(const expression *const l, const expression *const r);
  neq_expression(const neq_expression &orig) = delete;
  virtual ~neq_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const expression *const left;
  const expression *const right;
};
}
}