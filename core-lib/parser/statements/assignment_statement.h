#pragma once

#include "statement.h"
#include <vector>
#include <string>

namespace ratio
{

namespace ast
{

class expression;

class assignment_statement : public statement
{
public:
  assignment_statement(const std::vector<std::string> &is, const std::string &i, const expression *const e);
  assignment_statement(const assignment_statement &orig) = delete;
  virtual ~assignment_statement();

  void execute(const scope &scp, context &ctx) const override;

private:
  const std::vector<std::string> ids;
  const std::string id;
  const expression *const xpr;
};
}
}