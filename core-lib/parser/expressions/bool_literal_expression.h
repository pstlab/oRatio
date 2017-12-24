#pragma once

#include "expression.h"

namespace ratio
{

namespace ast
{

class bool_literal_expression : public expression
{
public:
  bool_literal_expression(const bool &l);
  bool_literal_expression(const bool_literal_expression &orig) = delete;
  virtual ~bool_literal_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const bool literal;
};
}
}