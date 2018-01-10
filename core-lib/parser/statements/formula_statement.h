#pragma once

#include "statement.h"
#include <string>
#include <vector>

namespace ratio
{

namespace ast
{

class expression;

class formula_statement : public statement
{
public:
  formula_statement(const bool &isf, const std::string &fn, const std::vector<std::string> &scp, const std::string &pn, const std::vector<std::pair<std::string, const expression *>> &assns);
  formula_statement(const formula_statement &orig) = delete;
  virtual ~formula_statement();

  void execute(const scope &scp, context &ctx) const override;

private:
  const bool is_fact;
  const std::string formula_name;
  const std::vector<std::string> formula_scope;
  const std::string predicate_name;
  const std::vector<std::pair<std::string, const expression *>> assignments;
};
}
}