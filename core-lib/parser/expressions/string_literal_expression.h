#pragma once

#include "expression.h"
#include <string>

namespace ratio
{

namespace ast
{

class string_literal_expression : public expression
{
public:
  string_literal_expression(const std::string &l);
  string_literal_expression(const string_literal_expression &orig) = delete;
  virtual ~string_literal_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const std::string literal;
};
}
}