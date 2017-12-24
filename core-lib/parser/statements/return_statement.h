#pragma once

#include "statement.h"

namespace ratio
{

namespace ast
{

class expression;

class return_statement : public statement
{
public:
  return_statement(const expression *const e);
  return_statement(const return_statement &orig) = delete;
  virtual ~return_statement();

  void execute(const scope &scp, context &ctx) const override;

private:
  const expression *const xpr;
};
}
}