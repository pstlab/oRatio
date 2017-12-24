#pragma once

namespace ratio
{

class scope;
class context;
class expr;

namespace ast
{

class expression
{
public:
  expression() {}
  expression(const expression &orig) = delete;
  virtual ~expression() {}

  virtual expr evaluate(const scope &scp, context &ctx) const = 0;
};
}
}