#include "solver.h"
#include "graph.h"
#include "var_flaw.h"
#include "atom_flaw.h"
#include "disjunction_flaw.h"
#include "composite_flaw.h"
#include "smart_type.h"
#include "atom.h"
#include "state_variable.h"
#include "reusable_resource.h"
#include "propositional_state.h"
#include "propositional_agent.h"
#ifdef BUILD_GUI
#include "solver_listener.h"
#endif
#include <cassert>

using namespace smt;

namespace ratio
{

solver::solver() : core(), theory(get_sat_core()), gr(*this) {}
solver::~solver() {}

void solver::init()
{
    read(std::vector<std::string>({"init.rddl"}));
    new_types({new state_variable(*this),
               new reusable_resource(*this),
               new propositional_state(*this),
               new propositional_agent(*this)});
}

void solver::solve()
{
    // we build the causal graph..
    gr.build();
    // TODO: add solving code here..
}

expr solver::new_enum(const type &tp, const std::unordered_set<item *> &allowed_vals)
{
    assert(allowed_vals.size() > 1);
    assert(tp.get_name().compare(BOOL_KEYWORD) != 0);
    assert(tp.get_name().compare(INT_KEYWORD) != 0);
    assert(tp.get_name().compare(REAL_KEYWORD) != 0);
    // we create a new enum expression..
    // notice that we do not enforce the exct_one constraint!
    var_expr xp = new var_item(*this, tp, get_ov_theory().new_var(std::unordered_set<var_value *>(allowed_vals.begin(), allowed_vals.end()), false));
    if (allowed_vals.size() > 1) // we create a new var flaw..
        gr.new_flaw(*new var_flaw(gr, gr.res, *xp));
    return xp;
}

void solver::new_fact(atom &atm)
{
    // we create a new atom flaw representing a fact..
    atom_flaw *af = new atom_flaw(gr, gr.res, atm, true);
    gr.new_flaw(*af);

    // we associate the flaw to the atom..
    reason.emplace(&atm, af);

    // we check if we need to notify the new fact to any smart types..
    if (&atm.get_type().get_scope() != this)
    {
        std::queue<type *> q;
        q.push(static_cast<type *>(&atm.get_type().get_scope()));
        while (!q.empty())
        {
            if (smart_type *st = dynamic_cast<smart_type *>(q.front()))
                st->new_fact(*af);
            for (const auto &st : q.front()->get_supertypes())
                q.push(st);
            q.pop();
        }
    }
}

void solver::new_goal(atom &atm)
{
    // we create a new atom flaw representing a goal..
    atom_flaw *af = new atom_flaw(gr, gr.res, atm, false);
    gr.new_flaw(*af);

    // we associate the flaw to the atom..
    reason.emplace(&atm, af);

    // we check if we need to notify the new goal to any smart types..
    if (&atm.get_type().get_scope() != this)
    {
        std::queue<type *> q;
        q.push(static_cast<type *>(&atm.get_type().get_scope()));
        while (!q.empty())
        {
            if (smart_type *st = dynamic_cast<smart_type *>(q.front()))
                st->new_goal(*af);
            for (const auto &st : q.front()->get_supertypes())
                q.push(st);
            q.pop();
        }
    }
}

void solver::new_disjunction(context &d_ctx, const disjunction &disj)
{
    // we create a new disjunction flaw..
    disjunction_flaw *df = new disjunction_flaw(gr, gr.res, d_ctx, disj);
    gr.new_flaw(*df);
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
