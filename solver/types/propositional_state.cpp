#include "propositional_state.h"
#include "solver.h"
#include "field.h"
#include "atom.h"
#include "atom_flaw.h"

namespace ratio
{

propositional_state::propositional_state(solver &slv) : smart_type(slv, slv, PROPOSITIONAL_STATE_NAME)
{
    new_constructors({new ps_constructor(*this)});
    new_predicates({new ps_predicate(*this)});
}
propositional_state::~propositional_state()
{
    // we clear the atom listeners..
    for (const auto &a : atoms)
        delete a.second;
}

std::vector<flaw *> propositional_state::get_flaws()
{
    std::vector<flaw *> flaws;
    if (to_check.empty()) // nothing has changed since last inconsistency check..
        return flaws;
    else
        return flaws;
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

propositional_state::ps_flaw::ps_flaw(solver &s, const std::set<atom *> &overlapping_atoms) : flaw(s, smart_type::get_resolvers(s, overlapping_atoms)), overlapping_atoms(overlapping_atoms) {}
propositional_state::ps_flaw::~ps_flaw() {}
void propositional_state::ps_flaw::compute_resolvers() {}

#ifdef BUILD_GUI
std::string propositional_state::ps_flaw::get_label() const
{
    return "φ" + std::to_string(get_phi()) + " ps-flaw";
}
#endif

propositional_state::order_resolver::order_resolver(solver &slv, const smt::var &r, ps_flaw &f, const atom &before, const atom &after) : resolver(slv, r, 0, f), before(before), after(after)
{
}
propositional_state::order_resolver::~order_resolver() {}
void propositional_state::order_resolver::apply() {}

#ifdef BUILD_GUI
std::string propositional_state::order_resolver::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " σ" + std::to_string(before.get_sigma()) + " <= σ" + std::to_string(after.get_sigma());
}
#endif

propositional_state::displace_resolver::displace_resolver(solver &slv, ps_flaw &f, const atom &a0, const atom &a1, const smt::lit &neq_lit) : resolver(slv, 0, f), a0(a0), a1(a1), neq_lit(neq_lit)
{
}
propositional_state::displace_resolver::~displace_resolver() {}
void propositional_state::displace_resolver::apply() { slv.get_sat_core().new_clause({smt::lit(get_rho(), false), neq_lit}); }

#ifdef BUILD_GUI
std::string propositional_state::displace_resolver::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " displ σ" + std::to_string(a0.get_sigma()) + ".τ != σ" + std::to_string(a1.get_sigma()) + ".τ";
}
#endif
} // namespace ratio