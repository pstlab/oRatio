#include "solver.h"
#ifdef BUILD_GUI
#include "solver_listener.h"
#endif

using namespace smt;

namespace ratio
{

solver::solver() : core(), theory(get_sat_core()), gr(*this) {}
solver::~solver() {}

void solver::init()
{
    read(std::vector<std::string>({"init.rddl"}));
    // TODO: add custom types..
}

void solver::solve()
{
    // TODO: add solving code here..
}

void solver::new_fact(atom &atm)
{
    // TODO: add code for creating a new fact flaw..
}

void solver::new_goal(atom &atm)
{
    // TODO: add code for creating a new goal flaw..
}

void solver::new_disjunction(context &d_ctx, const disjunction &disj)
{
    // TODO: add code for creating a new disjunction flaw..
}

void solver::take_decision(const smt::lit &ch)
{
    // TODO: add code for taking a decision..
}

void solver::next()
{
    // TODO: add code for moving to the next solution..
}

bool solver::propagate(const lit &p, std::vector<lit> &cnfl)
{
    return true;
}

bool solver::check(std::vector<lit> &cnfl)
{
    return true;
}

void solver::push()
{
}

void solver::pop()
{
}

#ifdef BUILD_GUI
void solver::fire_new_flaw(const flaw &f) const
{
    for (const auto &l : listeners)
        l->new_flaw(f);
}
void solver::fire_flaw_state_changed(const flaw &f) const
{
    for (const auto &l : listeners)
        l->flaw_state_changed(f);
}
void solver::fire_flaw_cost_changed(const flaw &f) const
{
    for (const auto &l : listeners)
        l->flaw_cost_changed(f);
}
void solver::fire_current_flaw(const flaw &f) const
{
    for (const auto &l : listeners)
        l->current_flaw(f);
}
void solver::fire_new_resolver(const resolver &r) const
{
    for (const auto &l : listeners)
        l->new_resolver(r);
}
void solver::fire_resolver_state_changed(const resolver &r) const
{
    for (const auto &l : listeners)
        l->resolver_state_changed(r);
}
void solver::fire_resolver_cost_changed(const resolver &r) const
{
    for (const auto &l : listeners)
        l->resolver_cost_changed(r);
}
void solver::fire_current_resolver(const resolver &r) const
{
    for (const auto &l : listeners)
        l->current_resolver(r);
}
void solver::fire_causal_link_added(const flaw &f, const resolver &r) const
{
    for (const auto &l : listeners)
        l->causal_link_added(f, r);
}
void solver::fire_state_changed() const
{
    for (const auto &l : listeners)
        l->state_changed();
}
#endif
} // namespace ratio
