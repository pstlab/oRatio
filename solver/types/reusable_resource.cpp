#include "reusable_resource.h"
#include "solver.h"
#include "atom.h"
#include "field.h"
#include "atom_flaw.h"
#include "combinations.h"
#include "riddle_parser.h"

using namespace smt;

namespace ratio
{

reusable_resource::reusable_resource(solver &slv) : smart_type(slv, slv, REUSABLE_RESOURCE_NAME)
{
    new_fields(*this, {new field(slv.get_type("real"), REUSABLE_RESOURCE_CAPACITY)});
    new_constructors({new rr_constructor(*this)});
    new_predicates({new use_predicate(*this)});
}

reusable_resource::~reusable_resource() {}

std::vector<flaw *> reusable_resource::get_flaws()
{
    std::vector<flaw *> flaws;
    if (to_check.empty()) // nothing has changed since last inconsistency check..
        return flaws;
    else
    {
        // we partition atoms for each reusable-resource they might insist on..
        std::unordered_map<item *, std::vector<atom *>> rr_instances;
        for (const auto &a : atoms)
            if (get_core().get_sat_core().value(a.first->get_sigma()) == True) // we filter out those atoms which are not strictly active..
            {
                expr c_scope = a.first->get(TAU);
                if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))
                {
                    for (const auto &val : get_core().get_ov_theory().value(enum_scope->ev))
                        if (to_check.find(static_cast<item *>(val)) != to_check.end())
                            rr_instances[static_cast<item *>(val)].push_back(a.first);
                }
                else
                    rr_instances[static_cast<item *>(&*c_scope)].push_back(a.first);
            }

        // we detect flaws for each of the instances..
        for (const auto &rr : rr_instances)
        {
            // for each pulse, the atoms starting at that pulse..
            std::map<inf_rational, std::set<atom *>> starting_atoms;
            // for each pulse, the atoms ending at that pulse..
            std::map<inf_rational, std::set<atom *>> ending_atoms;
            // all the pulses of the timeline..
            std::set<inf_rational> pulses;
            // the resource capacity..
            arith_expr capacity = rr.first->get(REUSABLE_RESOURCE_CAPACITY);
            inf_rational c_capacity = get_core().arith_value(capacity);

            for (const auto &a : rr.second)
            {
                arith_expr s_expr = a->get("start");
                arith_expr e_expr = a->get("end");
                inf_rational start = get_core().arith_value(s_expr);
                inf_rational end = get_core().arith_value(e_expr);
                starting_atoms[start].insert(a);
                ending_atoms[end].insert(a);
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
                inf_rational c_usage; // the concurrent resource usage..
                for (const auto &a : overlapping_atoms)
                {
                    arith_expr amount = a->get(REUSABLE_RESOURCE_USE_AMOUNT_NAME);
                    c_usage += get_core().arith_value(amount);
                }

                if (c_usage > c_capacity) // we have a 'peak'..
                    flaws.push_back(new rr_flaw(get_solver(), overlapping_atoms));
            }
        }

        to_check.clear();
        return flaws;
    }
}

void reusable_resource::new_fact(atom_flaw &f)
{
    // we apply interval-predicate whenever the fact becomes active..
    atom &atm = f.get_atom();
    set_ni(atm.get_sigma());
    static_cast<predicate &>(get_predicate(REUSABLE_RESOURCE_USE_PREDICATE_NAME)).apply_rule(atm);
    restore_ni();

    // we avoid unification..
    if (!get_core().get_sat_core().new_clause({lit(f.get_phi(), false), atm.get_sigma()}))
        throw std::runtime_error("the problem is unsolvable");

    atoms.push_back({&atm, new rr_atom_listener(*this, atm)});

    // we filter out those atoms which are not strictly active..
    if (get_core().get_sat_core().value(atm.get_sigma()) == True)
    {
        expr c_scope = atm.get(TAU);
        if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))              // the 'tau' parameter is a variable..
            for (const auto &val : get_core().get_ov_theory().value(enum_scope->ev)) // we check for all its allowed values..
                to_check.insert(static_cast<item *>(val));
        else // the 'tau' parameter is a constant..
            to_check.insert(&*c_scope);
    }
}

void reusable_resource::new_goal(atom_flaw &) { throw std::logic_error("it is not possible to define goals on a reusable resource.."); }

reusable_resource::rr_constructor::rr_constructor(reusable_resource &rr) : constructor(rr.get_core(), rr, {new field(rr.get_core().get_type(REAL_KEYWORD), REUSABLE_RESOURCE_CAPACITY)}, {{REUSABLE_RESOURCE_CAPACITY, {static_cast<const ast::expression *>(new ast::id_expression({REUSABLE_RESOURCE_CAPACITY}))}}}, {static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({REUSABLE_RESOURCE_CAPACITY})), static_cast<const ast::expression *>(new ast::real_literal_expression(0))))))}) {}
reusable_resource::rr_constructor::~rr_constructor() {}

reusable_resource::use_predicate::use_predicate(reusable_resource &rr) : predicate(rr.get_core(), rr, REUSABLE_RESOURCE_USE_PREDICATE_NAME, {new field(rr.get_core().get_type(REAL_KEYWORD), REUSABLE_RESOURCE_USE_AMOUNT_NAME), new field(rr, TAU)}, {static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({REUSABLE_RESOURCE_USE_AMOUNT_NAME})), static_cast<const ast::expression *>(new ast::real_literal_expression(0))))))}) { new_supertypes(*this, {&rr.get_core().get_predicate("IntervalPredicate")}); }
reusable_resource::use_predicate::~use_predicate() {}

reusable_resource::rr_atom_listener::rr_atom_listener(reusable_resource &rr, atom &atm) : atom_listener(atm), rr(rr) {}
reusable_resource::rr_atom_listener::~rr_atom_listener() {}

void reusable_resource::rr_atom_listener::something_changed()
{
    // we filter out those atoms which are not strictly active..
    if (atm.get_core().get_sat_core().value(atm.get_sigma()) == True)
    {
        expr c_scope = atm.get(TAU);
        if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))                  // the 'tau' parameter is a variable..
            for (const auto &val : atm.get_core().get_ov_theory().value(enum_scope->ev)) // we check for all its allowed values..
                rr.to_check.insert(static_cast<item *>(val));
        else // the 'tau' parameter is a constant..
            rr.to_check.insert(&*c_scope);
    }
}

reusable_resource::rr_flaw::rr_flaw(solver &slv, const std::set<atom *> &atms) : flaw(slv, smart_type::get_resolvers(slv, atms)), overlapping_atoms(atms) {}
reusable_resource::rr_flaw::~rr_flaw() {}

#ifdef BUILD_GUI
std::string reusable_resource::rr_flaw::get_label() const
{
    return "φ" + std::to_string(get_phi()) + "rr-flaw";
}
#endif

void reusable_resource::rr_flaw::compute_resolvers()
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

reusable_resource::order_resolver::order_resolver(solver &slv, const var &r, rr_flaw &f, const atom &before, const atom &after) : resolver(slv, r, 0, f), before(before), after(after) {}
reusable_resource::order_resolver::~order_resolver() {}

#ifdef BUILD_GUI
std::string reusable_resource::order_resolver::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " σ" + std::to_string(before.get_sigma()) + " <= σ" + std::to_string(after.get_sigma());
}
#endif

void reusable_resource::order_resolver::apply()
{
}

reusable_resource::displace_resolver::displace_resolver(solver &slv, rr_flaw &f, const atom &a0, const atom &a1, const lit &neq_lit) : resolver(slv, 0, f), a0(a0), a1(a1), neq_lit(neq_lit) {}
reusable_resource::displace_resolver::~displace_resolver() {}

#ifdef BUILD_GUI
std::string reusable_resource::displace_resolver::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " displ σ" + std::to_string(a0.get_sigma()) + ".τ != σ" + std::to_string(a1.get_sigma()) + ".τ";
}
#endif

void reusable_resource::displace_resolver::apply()
{
    slv.get_sat_core().new_clause({lit(get_rho(), false), neq_lit});
}
} // namespace ratio