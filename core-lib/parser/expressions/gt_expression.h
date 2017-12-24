#pragma once

#include "expression.h"

namespace ratio
{

namespace ast
{

class gt_expression : public expression
{
public:
  gt_expression(const expression *const l, const expression *const r);
  gt_expression(const gt_expression &orig) = delete;
  virtual ~gt_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const expression *const left;
  const expression *const right;
};
}
}