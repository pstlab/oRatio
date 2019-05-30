#include "state_variable.h"
#include "solver.h"
#include "atom.h"
#include "predicate.h"
#include "field.h"
#include "atom_flaw.h"

using namespace smt;

namespace ratio
{

state_variable::state_variable(solver &slv) : smart_type(slv, slv, STATE_VARIABLE_NAME) { new_constructors({new sv_constructor(*this)}); }

state_variable::~state_variable()
{
    // we clear the atom listeners..
    for (const auto &a : atoms)
        delete a.second;
}

std::vector<std::vector<std::pair<lit, double>>> state_variable::get_current_incs()
{
    std::vector<std::vector<std::pair<lit, double>>> incs;
    // we partition atoms for each state-variable they might insist on..
    std::unordered_map<item *, std::vector<atom *>> sv_instances;
    for (const auto &atm : atoms)
        if (get_core().get_sat_core().value(atm.first->get_sigma()) == True) // we filter out those which are not strictly active..
        {
            expr c_scope = atm.first->get(TAU);
            if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))
            {
                for (const auto &val : get_core().get_ov_theory().value(enum_scope->ev))
                    if (to_check.find(static_cast<item *>(val)) != to_check.end())
                        sv_instances[static_cast<item *>(val)].push_back(atm.first);
            }
            else if (to_check.find(static_cast<item *>(&*c_scope)) != to_check.end())
                sv_instances[static_cast<item *>(&*c_scope)].push_back(atm.first);
        }

    // we detect inconsistencies for each of the instances..
    for (const auto &sv : sv_instances)
    {
        // for each pulse, the atoms starting at that pulse..
        std::map<inf_rational, std::set<atom *>> starting_atoms;
        // for each pulse, the atoms ending at that pulse..
        std::map<inf_rational, std::set<atom *>> ending_atoms;
        // all the pulses of the timeline..
        std::set<inf_rational> pulses;

        for (const auto &atm : sv.second)
        {
            arith_expr s_expr = atm->get(START);
            arith_expr e_expr = atm->get(END);
            inf_rational start = get_core().arith_value(s_expr);
            inf_rational end = get_core().arith_value(e_expr);
            starting_atoms[start].insert(atm);
            ending_atoms[end].insert(atm);
            pulses.insert(start);
            pulses.insert(end);
        }

        std::set<atom *> overlapping_atoms;
        for (const auto &p : pulses)
        {
            const auto at_start_p = starting_atoms.find(p);
            if (at_start_p != starting_atoms.end())
                overlapping_atoms.insert(at_start_p->second.begin(), at_start_p->second.end());
            const auto at_end_p = ending_atoms.find(p);
            if (at_end_p != ending_atoms.end())
                for (const auto &a : at_end_p->second)
                    overlapping_atoms.erase(a);

            if (overlapping_atoms.size() > 1) // we have a 'peak'..
            {
                new_inc(overlapping_atoms);
                // TODO: add code for managing peaks..
            }
        }
    }
    return incs;
}

void state_variable::new_predicate(predicate &pred)
{
    // each state-variable predicate is also an interval-predicate..
    new_supertypes(pred, {&get_core().get_predicate("IntervalPredicate")});
    // each state-variable predicate has a tau parameter indicating on which state-variables the atoms insist on..
    new_fields(pred, {new field(static_cast<type &>(pred.get_scope()), TAU)});
}

void state_variable::new_fact(atom_flaw &f)
{
    // we apply interval-predicate whenever the fact becomes active..
    atom &atm = f.get_atom();
    set_ni(atm.get_sigma());
    get_core().get_predicate("IntervalPredicate").apply_rule(atm);
    restore_ni();

    // we store the variables for on-line flaw resolution..
    for (const auto &c_atm : atoms)
        store_variables(atm, *c_atm.first);

    // we store, for the fact, its atom listener..
    atoms.push_back({&atm, new sv_atom_listener(*this, atm)});

    // we filter out those atoms which are not strictly active..
    if (atm.get_core().get_sat_core().value(atm.get_sigma()) == True)
    {
        expr c_scope = atm.get(TAU);
        if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))                  // the 'tau' parameter is a variable..
            for (const auto &val : atm.get_core().get_ov_theory().value(enum_scope->ev)) // we check for all its allowed values..
                to_check.insert(static_cast<item *>(val));
        else // the 'tau' parameter is a constant..
            to_check.insert(&*c_scope);
    }
}

void state_variable::new_goal(atom_flaw &f)
{
    atom &atm = f.get_atom();

    // we store the variables for on-line flaw resolution..
    for (const auto &c_atm : atoms)
        store_variables(atm, *c_atm.first);

    // we store, for the goal, its atom listener..
    atoms.push_back({&atm, new sv_atom_listener(*this, atm)});

    // we filter out those atoms which are not strictly active..
    if (atm.get_core().get_sat_core().value(atm.get_sigma()) == True)
    {
        expr c_scope = atm.get(TAU);
        if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))                  // the 'tau' parameter is a variable..
            for (const auto &val : atm.get_core().get_ov_theory().value(enum_scope->ev)) // we check for all its allowed values..
                to_check.insert(static_cast<item *>(val));
        else // the 'tau' parameter is a constant..
            to_check.insert(&*c_scope);
    }
}

void state_variable::store_variables(atom &atm0, atom &atm1)
{
    expr a0_tau = atm0.get(TAU);
    expr a1_tau = atm1.get(TAU);
    item *a0_tau_itm = dynamic_cast<var_item *>(&*a0_tau);
    item *a1_tau_itm = dynamic_cast<var_item *>(&*a1_tau);
    if (a0_tau_itm || a1_tau_itm)
    {
        if (a0_tau_itm && a1_tau_itm)
        {
            // we have two non-singleton variables..
            var_item *a0_var_itm = static_cast<var_item *>(a0_tau_itm);
            var_item *a1_var_itm = static_cast<var_item *>(a1_tau_itm);
            std::unordered_set<var_value *> a0_vals = get_solver().enum_value(a0_var_itm);
            std::unordered_set<var_value *> a1_vals = get_solver().enum_value(a1_var_itm);

            bool found = false;
            for (const auto &v0 : a0_vals)
                if (a1_vals.find(v0) != a1_vals.end())
                {
                    found = true;
                    break;
                }
            if (found) // we store the ordering variables..
            {
#ifdef BUILD_GUI
                std::vector<std::pair<smt::lit, std::string>> chs({{get_solver().leq(atm0.get(END), atm1.get(START))->l, "σ" + std::to_string(atm0.get_sigma()) + " <= σ" + std::to_string(atm1.get_sigma())}, {get_solver().leq(atm1.get(END), atm0.get(START))->l, "σ" + std::to_string(atm1.get_sigma()) + " <= σ" + std::to_string(atm0.get_sigma())}});
                for (const auto &v0 : a0_vals)
                    if (a1_vals.find(v0) != a1_vals.end())
                        chs.push_back({get_solver().get_sat_core().new_conj({get_solver().get_ov_theory().allows(a0_var_itm->ev, *v0), smt::lit(get_solver().get_ov_theory().allows(a1_var_itm->ev, *v0), false)}), "σ" + std::to_string(atm0.get_sigma()) + ".τ ≠ σ" + std::to_string(atm1.get_sigma())});
#else
                std::vector<smt::lit> chs({get_solver().leq(atm0.get(END), atm1.get(START))->l, get_solver().leq(atm1.get(END), atm0.get(START))->l});
                for (const auto &v0 : a0_vals)
                    if (a1_vals.find(v0) != a1_vals.end())
                        chs.push_back(get_solver().get_sat_core().new_conj({get_solver().get_ov_theory().allows(a0_var_itm->ev, *v0), smt::lit(get_solver().get_ov_theory().allows(a1_var_itm->ev, *v0), false)}));
#endif
                set_choices(atm0, atm1, chs);
            }
        }
        else if (!a0_tau_itm)
        {
            var_item *a1_var_itm = static_cast<var_item *>(a1_tau_itm);
            std::unordered_set<var_value *> a1_vals = get_solver().enum_value(a1_var_itm);
            if (a1_vals.find(&*a0_tau) != a1_vals.end()) // we store the ordering variables..
#ifdef BUILD_GUI
                set_choices(atm0, atm1, std::vector<std::pair<smt::lit, std::string>>({{get_solver().leq(atm0.get(END), atm1.get(START))->l, "σ" + std::to_string(atm0.get_sigma()) + " <= σ" + std::to_string(atm1.get_sigma())}, {get_solver().leq(atm1.get(END), atm0.get(START))->l, "σ" + std::to_string(atm1.get_sigma()) + " <= σ" + std::to_string(atm0.get_sigma())}, {smt::lit(get_solver().get_ov_theory().allows(a1_var_itm->ev, *a0_tau), false), "σ" + std::to_string(atm0.get_sigma()) + ".τ ≠ σ" + std::to_string(atm1.get_sigma())}}));
#else
                set_choices(atm0, atm1, std::vector<smt::lit>({get_solver().leq(atm0.get(END), atm1.get(START))->l, get_solver().leq(atm1.get(END), atm0.get(START))->l, smt::lit(get_solver().get_ov_theory().allows(a1_var_itm->ev, *a0_tau), false)}));
#endif
        }
        else if (!a1_tau_itm)
        {
            var_item *a0_var_itm = static_cast<var_item *>(a0_tau_itm);
            std::unordered_set<var_value *> a0_vals = get_solver().enum_value(a0_var_itm);
            if (a0_vals.find(&*a1_tau) != a0_vals.end()) // we store the ordering variables..
#ifdef BUILD_GUI
                set_choices(atm0, atm1, std::vector<std::pair<smt::lit, std::string>>({{get_solver().leq(atm0.get(END), atm1.get(START))->l, "σ" + std::to_string(atm0.get_sigma()) + " <= σ" + std::to_string(atm1.get_sigma())}, {get_solver().leq(atm1.get(END), atm0.get(START))->l, "σ" + std::to_string(atm1.get_sigma()) + " <= σ" + std::to_string(atm0.get_sigma())}, {get_solver().get_ov_theory().allows(a0_var_itm->ev, *a1_tau), "σ" + std::to_string(atm0.get_sigma()) + ".τ ≠ σ" + std::to_string(atm1.get_sigma())}}));
#else
                set_choices(atm0, atm1, std::vector<smt::lit>({get_solver().leq(atm0.get(END), atm1.get(START))->l, get_solver().leq(atm1.get(END), atm0.get(START))->l, smt::lit(get_solver().get_ov_theory().allows(a0_var_itm->ev, *a1_tau), false)}));
#endif
        }
    }
    else if (&*a0_tau == &*a1_tau) // the two atoms are on the same state-variable: we store the ordering variables..
#ifdef BUILD_GUI
        set_choices(atm0, atm1, std::vector<std::pair<smt::lit, std::string>>({{get_solver().leq(atm0.get(END), atm1.get(START))->l, "σ" + std::to_string(atm0.get_sigma()) + " <= σ" + std::to_string(atm1.get_sigma())}, {get_solver().leq(atm1.get(END), atm0.get(START))->l, "σ" + std::to_string(atm1.get_sigma()) + " <= σ" + std::to_string(atm0.get_sigma())}}));
#else
        set_choices(atm0, atm1, std::vector<smt::lit>({get_solver().leq(atm0.get(END), atm1.get(START))->l, get_solver().leq(atm1.get(END), atm0.get(START))->l}));
#endif
}

state_variable::sv_constructor::sv_constructor(state_variable &sv) : constructor(sv.get_solver(), sv, {}, {}, {}) {}
state_variable::sv_constructor::~sv_constructor() {}

state_variable::sv_atom_listener::sv_atom_listener(state_variable &sv, atom &atm) : atom_listener(atm), sv(sv) { something_changed(); }
state_variable::sv_atom_listener::~sv_atom_listener() {}

void state_variable::sv_atom_listener::something_changed()
{
    // we filter out those atoms which are not strictly active..
    if (atm.get_core().get_sat_core().value(atm.get_sigma()) == True)
    {
        expr c_scope = atm.get(TAU);
        if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))                  // the 'tau' parameter is a variable..
            for (const auto &val : atm.get_core().get_ov_theory().value(enum_scope->ev)) // we check for all its allowed values..
                sv.to_check.insert(static_cast<item *>(val));
        else // the 'tau' parameter is a constant..
            sv.to_check.insert(&*c_scope);
    }
}
} // namespace ratio
