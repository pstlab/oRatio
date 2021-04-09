#pragma once

#include "scope.h"
#include "rational.h"

namespace riddle::ast
{
  class statement;
} // namespace riddle::ast

namespace ratio
{
  class context;

  class conjunction : public scope
  {
  public:
    conjunction(core &cr, scope &scp, const smt::rational &cst, const std::vector<const riddle::ast::statement *> &stmnts);
    conjunction(const conjunction &that) = delete;
    virtual ~conjunction();

    inline smt::rational get_cost() const noexcept { return cost; } // returns the cost of applying this conjunction..

    CORE_EXPORT void apply(context &ctx) const; // applies this conjunction within the given context..

  private:
    const smt::rational cost;
    const std::vector<const riddle::ast::statement *> statements;
  };
} // namespace ratio