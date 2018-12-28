#include "solver_listener.h"
#include "solver.h"
#include "graph.h"
#include <algorithm>

namespace ratio
{

solver_listener::solver_listener(solver &slv) : slv(slv) { slv.listeners.push_back(this); }
solver_listener::~solver_listener() { slv.listeners.erase(std::find(slv.listeners.begin(), slv.listeners.end(), this)); }

void solver_listener::new_flaw(const flaw &f)
{
    flaw_listeners.insert({&f, new flaw_listener(*this, f)});
    flaw_created(f);
}

void solver_listener::flaw_created(const flaw &) {}
void solver_listener::flaw_state_changed(const flaw &) {}
void solver_listener::current_flaw(const flaw &) {}

void solver_listener::new_resolver(const resolver &r)
{
    resolver_listeners.insert({&r, new resolver_listener(*this, r)});
    resolver_created(r);
}

void solver_listener::resolver_created(const resolver &) {}
void solver_listener::resolver_state_changed(const resolver &) {}
void solver_listener::resolver_cost_changed(const resolver &) {}
void solver_listener::current_resolver(const resolver &) {}

void solver_listener::causal_link_added(const flaw &, const resolver &) {}

void solver_listener::state_changed() {}

solver_listener::flaw_listener::flaw_listener(solver_listener &listener, const flaw &f) : sat_value_listener(listener.slv.get_sat_core()), listener(listener), f(f) { listen_sat(f.get_phi()); }
solver_listener::flaw_listener::~flaw_listener() {}
void solver_listener::flaw_listener::sat_value_change(const smt::var &) { listener.flaw_state_changed(f); }

solver_listener::resolver_listener::resolver_listener(solver_listener &listener, const resolver &r) : sat_value_listener(listener.slv.get_sat_core()), listener(listener), r(r) { listen_sat(r.get_rho()); }
solver_listener::resolver_listener::~resolver_listener() {}
void solver_listener::resolver_listener::sat_value_change(const smt::var &) { listener.resolver_state_changed(r); }
} // namespace ratio