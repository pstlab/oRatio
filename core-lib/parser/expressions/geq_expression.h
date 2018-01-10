#pragma once

#include "expression.h"

namespace ratio
{

namespace ast
{

class geq_expression : public expression
{
public:
  geq_expression(const expression *const l, const expression *const r);
  geq_expression(const geq_expression &orig) = delete;
  virtual ~geq_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const expression *const left;
  const expression *const right;
};
}
}