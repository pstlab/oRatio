#pragma once

#include "expression.h"
#include <vector>

namespace ratio
{

namespace ast
{

class multiplication_expression : public expression
{
public:
  multiplication_expression(const std::vector<expression *> &es);
  multiplication_expression(const multiplication_expression &orig) = delete;
  virtual ~multiplication_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const std::vector<expression *> expressions;
};
}
}