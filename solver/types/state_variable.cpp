#include "state_variable.h"
#include "solver.h"
#include "atom.h"
#include "predicate.h"
#include "field.h"
#include "atom_flaw.h"
#include "combinations.h"

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

std::vector<flaw *> state_variable::get_flaws()
{
    std::vector<flaw *> flaws;
    if (to_check.empty()) // nothing has changed since last inconsistency check..
        return flaws;
    else
    {
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
                else
                    sv_instances[static_cast<item *>(&*c_scope)].push_back(atm.first);
            }

        // we detect flaws for each of the instances..
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
                arith_expr s_expr = atm->get("start");
                arith_expr e_expr = atm->get("end");
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
                    flaws.push_back(new sv_flaw(get_solver(), overlapping_atoms));
            }
        }

        to_check.clear();
        return flaws;
    }
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

state_variable::sv_flaw::sv_flaw(solver &slv, const std::set<atom *> &atms) : flaw(slv, smart_type::get_resolvers(slv, atms)), overlapping_atoms(atms) {}
state_variable::sv_flaw::~sv_flaw() {}

#ifdef BUILD_GUI
std::string state_variable::sv_flaw::get_label() const
{
    return "φ" + std::to_string(get_phi()) + "sv-flaw";
}
#endif

void state_variable::sv_flaw::compute_resolvers()
{
    std::vector<std::vector<atom *>> cs = combinations(std::vector<atom *>(overlapping_atoms.begin(), overlapping_atoms.end()), 2);
    for (const auto &as : cs)
    {
        arith_expr a0_start = as[0]->get("start");
        arith_expr a0_end = as[0]->get("end");
        arith_expr a1_start = as[1]->get("start");
        arith_expr a1_end = as[1]->get("end");

        bool_expr a0_before_a1 = slv.leq(a0_end, a1_start);
        if (slv.bool_value(a0_before_a1) != False)
            add_resolver(*new order_resolver(slv, a0_before_a1->l.get_var(), *this, *as[0], *as[1]));
        bool_expr a1_before_a0 = slv.leq(a1_end, a0_start);
        if (slv.bool_value(a1_before_a0) != False)
            add_resolver(*new order_resolver(slv, a1_before_a0->l.get_var(), *this, *as[1], *as[0]));

        expr a0_tau = as[0]->get(TAU);
        expr a1_tau = as[1]->get(TAU);
        item *a0_tau_itm = dynamic_cast<var_item *>(&*a0_tau);
        item *a1_tau_itm = dynamic_cast<var_item *>(&*a1_tau);
        if (a0_tau_itm || a1_tau_itm)
        {
            if (!a0_tau_itm)
                a0_tau_itm = &*a0_tau;
            if (!a1_tau_itm)
                a1_tau_itm = &*a1_tau;
            add_resolver(*new displace_resolver(slv, *this, *as[0], *as[1], lit(a0_tau_itm->eq(*a1_tau_itm), false)));
        }
    }
}

state_variable::order_resolver::order_resolver(solver &slv, const var &r, sv_flaw &f, const atom &before, const atom &after) : resolver(slv, r, 0, f), before(before), after(after) {}
state_variable::order_resolver::~order_resolver() {}

#ifdef BUILD_GUI
std::string state_variable::order_resolver::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " σ" + std::to_string(before.get_sigma()) + " <= σ" + std::to_string(after.get_sigma());
}
#endif

void state_variable::order_resolver::apply()
{
}

state_variable::displace_resolver::displace_resolver(solver &slv, sv_flaw &f, const atom &a0, const atom &a1, const lit &neq_lit) : resolver(slv, 0, f), a0(a0), a1(a1), neq_lit(neq_lit) {}
state_variable::displace_resolver::~displace_resolver() {}

#ifdef BUILD_GUI
std::string state_variable::displace_resolver::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " displ σ" + std::to_string(a0.get_sigma()) + ".τ != σ" + std::to_string(a1.get_sigma()) + ".τ";
}
#endif

void state_variable::displace_resolver::apply()
{
    slv.get_sat_core().new_clause({lit(get_rho(), false), neq_lit});
}
} // namespace ratio