#pragma once

#include "expression.h"
#include <vector>

namespace ratio
{

namespace ast
{

class function_expression : public expression
{
public:
  function_expression(const std::vector<std::string> &is, const std::string &fn, const std::vector<expression *> &es);
  function_expression(const function_expression &orig) = delete;
  virtual ~function_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const std::vector<std::string> ids;
  const std::string function_name;
  const std::vector<expression *> expressions;
};
}
}