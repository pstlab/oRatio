#pragma once

#include "type_declaration.h"

namespace ratio
{

namespace ast
{

class expression;

class typedef_declaration : public type_declaration
{
public:
  typedef_declaration(const std::string &n, const std::string &pt, const expression *const e);
  typedef_declaration(const typedef_declaration &orig) = delete;
  virtual ~typedef_declaration();

  void declare(scope &scp) const override;

private:
  const std::string primitive_type;
  const expression *const xpr;
};
}
}