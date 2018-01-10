#pragma once

#include <vector>
#include <string>

namespace ratio
{

class scope;

namespace ast
{

class variable_declaration;

class field_declaration
{
public:
  field_declaration(const std::vector<std::string> &tp, const std::vector<variable_declaration *> &ds);
  field_declaration(const field_declaration &orig) = delete;
  virtual ~field_declaration();

  void refine(scope &scp) const;

private:
  const std::vector<std::string> field_type;
  const std::vector<variable_declaration *> declarations;
};
}
}