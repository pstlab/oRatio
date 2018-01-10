#pragma once

#include "expression.h"
#include <vector>

namespace ratio
{

namespace ast
{

class subtraction_expression : public expression
{
public:
  subtraction_expression(const std::vector<expression *> &es);
  subtraction_expression(const subtraction_expression &orig) = delete;
  virtual ~subtraction_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const std::vector<expression *> expressions;
};
}
}