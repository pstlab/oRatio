#pragma once

#include "sat_core.h"
#include <stack>

namespace smt
{
  class sat_stack
  {
  public:
    SMT_EXPORT sat_stack();
    sat_stack(const sat_stack &orig) = delete;
    SMT_EXPORT ~sat_stack();

    SMT_EXPORT void push() noexcept;
    SMT_EXPORT void pop() noexcept;

    SMT_EXPORT sat_core &top() noexcept { return stack.top(); }
    SMT_EXPORT bool empty() const noexcept { return stack.empty(); }
    SMT_EXPORT size_t size() const noexcept { return stack.size(); }

  private:
    std::stack<sat_core> stack;
  };
} // namespace smt