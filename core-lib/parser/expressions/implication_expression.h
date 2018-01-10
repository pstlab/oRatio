#pragma once

#include "expression.h"

namespace ratio
{

namespace ast
{

class implication_expression : public expression
{
public:
  implication_expression(const expression *const l, const expression *const r);
  implication_expression(const implication_expression &orig) = delete;
  virtual ~implication_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const expression *const left;
  const expression *const right;
};
}
}