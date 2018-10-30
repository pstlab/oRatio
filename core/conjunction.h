#pragma once

#include "scope.h"
#include "rational.h"

namespace ratio
{

class context;

class conjunction : public scope
{
  public:
    conjunction(core &cr, scope &scp, const smt::rational &cst);
    conjunction(const conjunction &that) = delete;
    virtual ~conjunction();

    smt::rational get_cost() const { return cost; } // returns the cost of applying this conjunction..

    void apply(context &ctx) const; // applies this conjunction within the given context..

  private:
    const smt::rational cost;
};
} // namespace ratio
