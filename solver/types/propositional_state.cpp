#include "propositional_state.h"
#include "solver.h"
#include "field.h"
#include "atom.h"
#include "atom_flaw.h"

using namespace smt;

namespace ratio
{

propositional_state::propositional_state(solver &slv) : smart_type(slv, slv, PROPOSITIONAL_STATE_NAME)
{
    new_constructors({new ps_constructor(*this)});
    new_predicates({new ps_predicate(*this)}, false);
}
propositional_state::~propositional_state()
{
    // we clear the atom listeners..
    for (const auto &a : atoms)
        delete a.second;
}

std::vector<std::pair<lit, double>> propositional_state::get_current_incs()
{
    std::vector<std::pair<lit, double>> incs;
    // TODO: add code for finding inconsistencies here..
    return incs;
}

void propositional_state::new_predicate(predicate &pred)
{
    new_supertypes(pred, {&get_predicate(PROPOSITIONAL_STATE_PREDICATE_NAME)});
    new_fields(pred, {new field(static_cast<type &>(pred.get_scope()), TAU)});
}

void propositional_state::new_fact(atom_flaw &f)
{
    // we apply interval-predicate if the fact becomes active..
    atom &atm = f.get_atom();
    set_ni(atm.get_sigma());
    static_cast<predicate &>(get_predicate(PROPOSITIONAL_STATE_PREDICATE_NAME)).apply_rule(atm);
    restore_ni();

    atoms.push_back({&atm, new ps_atom_listener(*this, atm)});
    to_check.insert(&atm);
}

void propositional_state::new_goal(atom_flaw &f)
{
    atom &atm = f.get_atom();
    atoms.push_back({&atm, new ps_atom_listener(*this, atm)});
    to_check.insert(&atm);
}

propositional_state::ps_constructor::ps_constructor(propositional_state &ps) : constructor(ps.get_solver(), ps, {}, {}, {}) {}
propositional_state::ps_constructor::~ps_constructor() {}

propositional_state::ps_predicate::ps_predicate(propositional_state &ps) : predicate(ps.get_solver(), ps, PROPOSITIONAL_STATE_PREDICATE_NAME, {new field(ps.get_solver().get_type("bool"), PROPOSITIONAL_STATE_POLARITY_NAME)}, {}) { new_supertypes({&ps.get_core().get_predicate("IntervalPredicate")}); }
propositional_state::ps_predicate::~ps_predicate() {}

propositional_state::ps_atom_listener::ps_atom_listener(propositional_state &ps, atom &atm) : atom_listener(atm), ps(ps) {}
propositional_state::ps_atom_listener::~ps_atom_listener() {}
void propositional_state::ps_atom_listener::something_changed() { ps.to_check.insert(&atm); }
} // namespace ratio