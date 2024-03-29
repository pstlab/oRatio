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
    agent::agent(solver &slv) : smart_type(slv, slv, AGENT_NAME), timelines_extractor() { new_constructors({new agnt_constructor(*this)}); }

    std::vector<std::vector<std::pair<lit, double>>> agent::get_current_incs()
    {
        std::vector<std::vector<std::pair<lit, double>>> incs;
        // TODO: add code for finding inconsistencies here..
        return incs;
    }

    void agent::new_predicate(predicate &pred) noexcept
    {
        assert(get_solver().is_impulse(pred) || get_solver().is_interval(pred));
        // each agent predicate has a tau parameter indicating on which agents the atoms insist on..
        new_fields(pred, {new field(static_cast<type &>(pred.get_scope()), TAU)});
    }

    void agent::new_atom(atom_flaw &f)
    {
        atom &atm = f.get_atom();
        if (f.is_fact)
        {
            set_ni(lit(atm.get_sigma()));
            if (get_solver().get_impulse().is_assignable_from(atm.get_type())) // we apply impulse-predicate whenever the fact becomes active..
                get_solver().get_impulse().apply_rule(atm);
            else // we apply interval-predicate whenever the fact becomes active..
                get_solver().get_interval().apply_rule(atm);
            restore_ni();
        }

        atoms.emplace_back(&atm, new agnt_atom_listener(*this, atm));
        to_check.insert(&atm);
    }

    agent::agnt_constructor::agnt_constructor(agent &agnt) : constructor(agnt, {}, {}, {}) {}
    agent::agnt_constructor::~agnt_constructor() {}

    agent::agnt_atom_listener::agnt_atom_listener(agent &agnt, atom &atm) : atom_listener(atm), agnt(agnt) {}
    agent::agnt_atom_listener::~agnt_atom_listener() {}
    void agent::agnt_atom_listener::something_changed() { agnt.to_check.insert(&atm); }

    json agent::extract_timelines() const noexcept
    {
        std::vector<json> tls;
        // we partition atoms for each state-variable they might insist on..
        std::unordered_map<const item *, std::vector<atom *>> agnt_instances;
        for ([[maybe_unused]] const auto &[atm, atm_lstnr] : atoms)
            if (get_core().get_sat_core().value(atm->get_sigma()) == True) // we filter out those which are not strictly active..
            {
                expr c_scope = atm->get(TAU);
                if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))
                {
                    for (const auto &val : get_core().get_ov_theory().value(enum_scope->ev))
                        agnt_instances[static_cast<const item *>(val)].emplace_back(atm);
                }
                else
                    agnt_instances[static_cast<item *>(&*c_scope)].emplace_back(atm);
            }

        for (const auto &[agnt, atms] : agnt_instances)
        {
            json tl;
            tl->set("id", new long_val(agnt->get_id()));
#if defined(VERBOSE_LOG) || defined(BUILD_LISTENERS)
            tl->set("name", new string_val(get_core().guess_name(*agnt)));
#endif
            tl->set("type", new string_val(AGENT_NAME));

            // for each pulse, the atoms starting at that pulse..
            std::map<inf_rational, std::set<atom *>> starting_atoms;
            // all the pulses of the timeline..
            std::set<inf_rational> pulses;

            for (const auto &atm : atms)
            {
                arith_expr s_expr = get_solver().is_impulse(*atm) ? atm->get(RATIO_AT) : atm->get(RATIO_START);
                inf_rational start = get_core().arith_value(s_expr);
                starting_atoms[start].insert(atm);
                pulses.insert(start);
            }

            std::vector<json> j_atms;
            for (const auto &p : pulses)
                for (const auto &atm : starting_atoms.at(p))
                    j_atms.emplace_back(new long_val(atm->get_id()));
            tl->set("values", new array_val(j_atms));
            tls.emplace_back(tl);
        }
        return new array_val(tls);
    }
} // namespace ratio