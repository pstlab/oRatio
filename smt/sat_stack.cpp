#include "sat_stack.h"
#include "theory.h"
#include "sat_value_listener.h"
#include <cassert>

namespace smt
{
    SMT_EXPORT sat_stack::sat_stack() { stack.resize(1); }

    SMT_EXPORT void sat_stack::push() noexcept
    {
        stack.push_back(stack.back());
        for (auto &th : stack.back().theories)
        {
            th->sat = &stack.back();
            th->push();
        }
        for (auto &l : stack.back().listeners)
            l->sat = &stack.back();
    }

    SMT_EXPORT void sat_stack::pop() noexcept
    {
        assert(stack.size() > 1);
        std::vector<lit> trail = stack.back().trail;
        stack.pop_back();
        for (auto &th : stack.back().theories)
        {
            th->sat = &stack.back();
            th->pop();
        }
        for (auto &l : stack.back().listeners)
            l->sat = &stack.back();

        for (const auto &p : trail)
            if (const auto at_p = stack.back().listening.find(variable(p)); at_p != stack.back().listening.cend())
                for (const auto &l : at_p->second)
                    l->sat_value_change(variable(p));
    }
} // namespace smt