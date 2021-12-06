#include "deliberative_solver_listener.h"

using namespace ratio;

namespace ratio
{
    deliberative_solver_listener::deliberative_solver_listener(solver &s) : solver_listener(s) {}
    deliberative_solver_listener::~deliberative_solver_listener() {}

    void deliberative_solver_listener::flaw_created(const flaw &f)
    {
    }
    void deliberative_solver_listener::flaw_state_changed(const flaw &f)
    {
    }
    void deliberative_solver_listener::flaw_cost_changed(const flaw &f)
    {
    }
    void deliberative_solver_listener::flaw_position_changed(const flaw &f)
    {
    }
    void deliberative_solver_listener::current_flaw(const flaw &f)
    {
    }

    void deliberative_solver_listener::resolver_created(const resolver &r)
    {
    }
    void deliberative_solver_listener::resolver_state_changed(const resolver &r)
    {
    }
    void deliberative_solver_listener::current_resolver(const resolver &r)
    {
    }

    void deliberative_solver_listener::causal_link_added(const flaw &f, const resolver &r)
    {
    }
} // namespace ratio
