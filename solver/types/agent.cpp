#include "agent.h"
#include "solver.h"
#include "atom.h"
#include "predicate.h"
#include "field.h"
#include "atom_flaw.h"
#include <assert.h>

using namespace smt;

namespace ratio
{
    agent::agent(solver &slv) : smart_type(slv, slv, AGENT_NAME), int_pred(slv.get_predicate("Interval")), imp_pred(slv.get_predicate("Impulse")) { new_constructors({new agnt_constructor(*this)}); }
    agent::~agent() {}

    std::vector<std::vector<std::pair<lit, double>>> agent::get_current_incs()
    {
        std::vector<std::vector<std::pair<lit, double>>> incs;
        // TODO: add code for finding inconsistencies here..
        return incs;
    }

    void agent::new_predicate(predicate &pred) noexcept
    { // each agent predicate has a tau parameter indicating on which agents the atoms insist on..
        new_fields(pred, {new field(static_cast<type &>(pred.get_scope()), TAU)});
    }

    void agent::new_atom(atom_flaw &f)
    {
        atom &atm = f.get_atom();
        if (f.is_fact)
        {
            set_ni(lit(atm.get_sigma()));
            if (imp_pred.is_assignable_from(atm.get_type())) // we apply impulse-predicate whenever the fact becomes active..
                imp_pred.apply_rule(atm);
            else // we apply interval-predicate whenever the fact becomes active..
                int_pred.apply_rule(atm);
            restore_ni();
        }

        atoms.push_back({&atm, new agnt_atom_listener(*this, atm)});
        to_check.insert(&atm);
    }

    agent::agnt_constructor::agnt_constructor(agent &agnt) : constructor(agnt.get_solver(), agnt, {}, {}, {}) {}
    agent::agnt_constructor::~agnt_constructor() {}

    agent::agnt_atom_listener::agnt_atom_listener(agent &agnt, atom &atm) : atom_listener(atm), agnt(agnt) {}
    agent::agnt_atom_listener::~agnt_atom_listener() {}
    void agent::agnt_atom_listener::something_changed() { agnt.to_check.insert(&atm); }
} // namespace ratio