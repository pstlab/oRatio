#pragma once

#include "expression.h"

namespace ratio
{

namespace ast
{

class range_expression : public expression
{
public:
  range_expression(const expression *const min_e, const expression *const max_e);
  range_expression(const range_expression &orig) = delete;
  virtual ~range_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const expression *const min_xpr;
  const expression *const max_xpr;
};
}
}