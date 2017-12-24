#pragma once

#include "statement.h"
#include <vector>

namespace ratio
{

namespace ast
{

class expression;

class local_field_statement : public statement
{
public:
  local_field_statement(const std::vector<std::string> &ft, const std::string &n, const expression *const e = nullptr);
  local_field_statement(const local_field_statement &orig) = delete;
  virtual ~local_field_statement();

  void execute(const scope &scp, context &ctx) const override;

private:
  const std::vector<std::string> field_type;
  const std::string name;
  const expression *const xpr;
};
}
}