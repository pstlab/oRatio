#pragma once

#include "expression.h"
#include <vector>

namespace ratio
{

namespace ast
{

class exct_one_expression : public expression
{
public:
  exct_one_expression(const std::vector<expression *> &es);
  exct_one_expression(const exct_one_expression &orig) = delete;
  virtual ~exct_one_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const std::vector<expression *> expressions;
};
}
}