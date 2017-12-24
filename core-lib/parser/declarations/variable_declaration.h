#pragma once

#include <string>

namespace lucy
{

namespace ast
{

class expression;
class field_declaration;

class variable_declaration
{
  friend class field_declaration;

public:
  variable_declaration(const std::string &n, const expression *const e = nullptr);
  variable_declaration(const variable_declaration &orig) = delete;
  virtual ~variable_declaration();

private:
  const std::string name;
  const expression *const xpr;
};
}
}