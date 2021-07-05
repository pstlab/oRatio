#include "sat_stack.h"
#include "theory.h"
#include "sat_value_listener.h"
#include <cassert>

namespace smt
{
    SMT_EXPORT sat_stack::sat_stack() { stack.emplace(); }
    SMT_EXPORT sat_stack::~sat_stack() {}

    SMT_EXPORT void sat_stack::push() noexcept
    {
        stack.push(stack.top());
        for (auto &th : stack.top().theories)
            th->sat = &stack.top();
        for (auto &l : stack.top().listeners)
            l->sat = &stack.top();
    }

    SMT_EXPORT void sat_stack::pop() noexcept
    {
        assert(stack.size() > 1);
        std::vector<lit> trail = stack.top().trail;
        stack.pop();
        for (auto &th : stack.top().theories)
            th->sat = &stack.top();
        for (auto &l : stack.top().listeners)
            l->sat = &stack.top();

        for (const auto &p : trail)
            if (const auto at_p = stack.top().listening.find(variable(p)); at_p != stack.top().listening.cend())
                for (const auto &l : at_p->second)
                    l->sat_value_change(variable(p));
    }
} // namespace smt