#pragma once

#include "scope.h"
#include "rational.h"

namespace ratio
{

namespace ast
{
class statement;
}

class context;

class conjunction : public scope
{
public:
  conjunction(core &cr, scope &scp, const smt::rational &cst, const std::vector<const ast::statement *> &stmnts);
  conjunction(const conjunction &that) = delete;
  virtual ~conjunction();

  smt::rational get_cost() const { return cost; }

  void apply(context &ctx) const;

private:
  const smt::rational cost;
  const std::vector<const ast::statement *> statements;
};
}