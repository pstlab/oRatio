#pragma once

#include "expression.h"
#include <vector>

namespace ratio
{

namespace ast
{

class conjunction_expression : public expression
{
public:
  conjunction_expression(const std::vector<expression *> &es);
  conjunction_expression(const conjunction_expression &orig) = delete;
  virtual ~conjunction_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const std::vector<expression *> expressions;
};
}
}