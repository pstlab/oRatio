#pragma once

#include "expression.h"
#include <vector>
#include <string>

namespace ratio
{

namespace ast
{

class constructor_expression : public expression
{
public:
  constructor_expression(const std::vector<std::string> &it, const std::vector<expression *> &es);
  constructor_expression(const constructor_expression &orig) = delete;
  virtual ~constructor_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const std::vector<std::string> instance_type;
  const std::vector<expression *> expressions;
};
}
}