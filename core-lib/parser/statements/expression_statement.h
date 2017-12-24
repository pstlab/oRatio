#pragma once

#include "statement.h"

namespace ratio
{

namespace ast
{

class expression;

class expression_statement : public statement
{
public:
  expression_statement(const expression *const e);
  expression_statement(const expression_statement &orig) = delete;
  virtual ~expression_statement();

  void execute(const scope &scp, context &ctx) const override;

private:
  const expression *const xpr;
};
}
}