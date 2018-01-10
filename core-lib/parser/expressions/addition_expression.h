#pragma once

#include "expression.h"
#include <vector>

namespace ratio
{

namespace ast
{

class addition_expression : public expression
{
public:
  addition_expression(const std::vector<expression *> &es);
  addition_expression(const addition_expression &orig) = delete;
  virtual ~addition_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const std::vector<expression *> expressions;
};
}
}