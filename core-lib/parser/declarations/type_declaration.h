#pragma once

#include <string>

namespace ratio
{

class scope;

namespace ast
{

class type_declaration
{
public:
  type_declaration(const std::string &n);
  type_declaration(const type_declaration &orig) = delete;
  virtual ~type_declaration();

  virtual void declare(scope &) const {}
  virtual void refine(scope &) const {}

protected:
  const std::string name;
};
}
}