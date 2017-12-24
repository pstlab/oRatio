#pragma once

#include <string>

namespace ratio
{

class type;

namespace ast
{
class expression;
}

class field
{
public:
  field(const type &tp, const std::string &name, const ast::expression *const e = nullptr, bool synthetic = false) : tp(tp), name(name), xpr(e), synthetic(synthetic) {}
  virtual ~field() {}

public:
  const type &tp;
  const std::string name;
  const ast::expression *const xpr;
  const bool synthetic;
};
}