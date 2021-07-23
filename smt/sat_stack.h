#pragma once

#include "sat_core.h"
#include <stack>

namespace smt
{
  class sat_stack final
  {
  public:
    SMT_EXPORT sat_stack();
    sat_stack(const sat_stack &orig) = delete;

    SMT_EXPORT void push() noexcept;
    SMT_EXPORT void pop() noexcept;

    SMT_EXPORT sat_core &top() noexcept { return stack.top(); }
    SMT_EXPORT bool empty() const noexcept { return stack.empty(); }
    SMT_EXPORT size_t size() const noexcept { return stack.size(); }

    inline lbool value(const var &x) const noexcept { return stack.top().value(x); }
    inline lbool value(const lit &p) const noexcept { return stack.top().value(p); }

  private:
    std::stack<sat_core> stack;
  };
} // namespace smt