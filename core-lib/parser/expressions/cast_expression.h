#pragma once

#include "expression.h"
#include <vector>
#include <string>

namespace ratio
{

namespace ast
{

class cast_expression : public expression
{
public:
  cast_expression(const std::vector<std::string> &tp, const expression *const e);
  cast_expression(const cast_expression &orig) = delete;
  virtual ~cast_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const std::vector<std::string> cast_to_type;
  const expression *const xpr;
};
}
}