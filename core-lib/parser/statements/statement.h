#pragma once

namespace ratio
{

class scope;
class context;

namespace ast
{

class statement
{
public:
  statement() {}
  statement(const statement &orig) = delete;
  virtual ~statement() {}

  virtual void execute(const scope &scp, context &ctx) const = 0;
};
}
}