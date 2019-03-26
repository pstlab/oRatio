#include "solver.h"

using namespace smt;

namespace ratio
{

solver::solver() : core(), theory(get_sat_core()) {}
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
}

void solver::new_goal(atom &atm)
{
}

void solver::new_disjunction(context &d_ctx, const disjunction &disj)
{
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
} // namespace ratio
