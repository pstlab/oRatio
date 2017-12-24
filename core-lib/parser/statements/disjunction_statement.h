#pragma once

#include "statement.h"
#include <vector>

namespace ratio
{

namespace ast
{

class statement;
class expression;

class disjunction_statement : public statement
{
public:
  disjunction_statement(const std::vector<std::pair<std::vector<const statement *>, const expression *const>> &conjs);
  disjunction_statement(const disjunction_statement &orig) = delete;
  virtual ~disjunction_statement();

  void execute(const scope &scp, context &ctx) const override;

private:
  const std::vector<std::pair<std::vector<const statement *>, const expression *const>> conjunctions;
};
}
}