#pragma once

#include "scope.h"
#include "rational.h"

using namespace smt;

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
  conjunction(core &cr, scope &scp, const rational &cst, const std::vector<const ast::statement *> &stmnts);
  conjunction(const conjunction &that) = delete;
  virtual ~conjunction();

  rational get_cost() const { return cost; }

  void apply(context &ctx) const;

private:
  const rational cost;
  const std::vector<const ast::statement *> statements;
};
}