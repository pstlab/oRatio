#pragma once

#include "expression.h"
#include <vector>

namespace ratio
{

namespace ast
{

class disjunction_expression : public expression
{
public:
  disjunction_expression(const std::vector<expression *> &es);
  disjunction_expression(const disjunction_expression &orig) = delete;
  virtual ~disjunction_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const std::vector<expression *> expressions;
};
}
}