#include "propositional_state.h"
#include "solver.h"
#include "field.h"
#include "atom.h"
#include "atom_flaw.h"
#include "riddle_lexer.h"

using namespace smt;

namespace ratio
{
    propositional_state::propositional_state(solver &slv) : smart_type(slv, slv, PROPOSITIONAL_STATE_NAME), int_pred(slv.get_predicate("Interval"))
    {
        new_constructors({new ps_constructor(*this)});
        new_predicates({new ps_predicate(*this)}, false);
    }
    propositional_state::~propositional_state()
    {
        // we clear the atom listeners..
        for (const auto &[atm, lstnr] : atoms)
            delete lstnr;
    }

    std::vector<std::vector<std::pair<lit, double>>> propositional_state::get_current_incs()
    {
        std::vector<std::vector<std::pair<lit, double>>> incs;
        // TODO: add code for finding inconsistencies here..
        return incs;
    }

    void propositional_state::new_predicate(predicate &pred) noexcept
    {
        new_supertypes(pred, {&get_predicate(PROPOSITIONAL_STATE_PREDICATE_NAME), const_cast<predicate *>(&int_pred)});
        new_fields(pred, {new field(static_cast<type &>(pred.get_scope()), TAU)});
    }

    void propositional_state::new_atom(atom_flaw &f)
    {
        atom &atm = f.get_atom();
        if (f.is_fact)
        { // we apply interval-predicate if the fact becomes active..
            set_ni(lit(atm.get_sigma()));
            static_cast<predicate &>(get_predicate(PROPOSITIONAL_STATE_PREDICATE_NAME)).apply_rule(atm);
            restore_ni();
        }

        // we store, for the atom, its atom listener..
        atoms.push_back({&atm, new ps_atom_listener(*this, atm)});
        to_check.insert(&atm);
    }

    propositional_state::ps_constructor::ps_constructor(propositional_state &ps) : constructor(ps.get_solver(), ps, {}, {}, {}) {}
    propositional_state::ps_constructor::~ps_constructor() {}

    propositional_state::ps_predicate::ps_predicate(propositional_state &ps) : predicate(ps.get_solver(), ps, PROPOSITIONAL_STATE_PREDICATE_NAME, {new field(ps.get_solver().get_type(BOOL_KEYWORD), PROPOSITIONAL_STATE_POLARITY_NAME)}, {}) {}
    propositional_state::ps_predicate::~ps_predicate() {}

    propositional_state::ps_atom_listener::ps_atom_listener(propositional_state &ps, atom &atm) : atom_listener(atm), ps(ps) {}
    propositional_state::ps_atom_listener::~ps_atom_listener() {}
    void propositional_state::ps_atom_listener::something_changed() { ps.to_check.insert(&atm); }
} // namespace ratio