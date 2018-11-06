#include "solver.h"
#include <cassert>

using namespace smt;

namespace ratio
{

solver::solver() : core(), theory(get_sat_core()) {}
solver::~solver() {}

void solver::solve() {}

expr solver::new_enum(const type &tp, const std::unordered_set<item *> &allowed_vals)
{
    // we create a new enum expression..
    var_expr xp = core::new_enum(tp, allowed_vals);
    // TODO: create a new enum flaw..
    return xp;
}

void solver::new_fact(atom &atm)
{
    // TODO: create a new fact flaw..
}

void solver::new_goal(atom &atm)
{
    // TODO: create a new goal flaw..
}

void solver::new_disjunction(context &d_ctx, const disjunction &disj)
{
    // TODO: create a new disjunction flaw..
}

bool solver::propagate(const lit &p, std::vector<lit> &cnfl)
{
    // TODO: propagate flaws and resolvers addition/removal..
    return true;
}

bool solver::check(std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    return true;
}

void solver::push() {}

void solver::pop() {}
} // namespace ratio
