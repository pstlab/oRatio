#include "propositional_agent.h"
#include "solver.h"
#include "atom.h"
#include "atom_flaw.h"

using namespace smt;

namespace ratio
{

propositional_agent::propositional_agent(solver &slv) : smart_type(slv, slv, PROPOSITIONAL_AGENT_NAME) { new_constructors({new agnt_constructor(*this)}); }
propositional_agent::~propositional_agent() {}

std::vector<std::vector<std::pair<lit, double>>> propositional_agent::get_current_incs()
{
    std::vector<std::vector<std::pair<lit, double>>> incs;
    // TODO: add code for finding inconsistencies here..
    return incs;
}

void propositional_agent::new_fact(atom_flaw &) { throw std::logic_error("it is not possible to define facts on propositional agents.."); }

void propositional_agent::new_goal(atom_flaw &f)
{
    atom &atm = f.get_atom();
    atoms.push_back({&atm, new agnt_atom_listener(*this, atm)});
    to_check.insert(&atm);
}

propositional_agent::agnt_constructor::agnt_constructor(propositional_agent &agnt) : constructor(agnt.get_solver(), agnt, {}, {}, {}) {}
propositional_agent::agnt_constructor::~agnt_constructor() {}

propositional_agent::agnt_atom_listener::agnt_atom_listener(propositional_agent &agnt, atom &atm) : atom_listener(atm), agnt(agnt) {}
propositional_agent::agnt_atom_listener::~agnt_atom_listener() {}
void propositional_agent::agnt_atom_listener::something_changed() { agnt.to_check.insert(&atm); }
} // namespace ratio