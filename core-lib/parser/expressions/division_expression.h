#pragma once

#include "expression.h"
#include <vector>

namespace ratio
{

namespace ast
{

class division_expression : public expression
{
public:
  division_expression(const std::vector<expression *> &es);
  division_expression(const division_expression &orig) = delete;
  virtual ~division_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const std::vector<expression *> expressions;
};
}
}