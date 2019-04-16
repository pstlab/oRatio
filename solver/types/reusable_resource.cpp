#include "reusable_resource.h"
#include "solver.h"
#include "atom.h"
#include "field.h"
#include "atom_flaw.h"
#include "combinations.h"
#include "riddle_parser.h"
#include <stdexcept>

using namespace smt;

namespace ratio
{

reusable_resource::reusable_resource(solver &slv) : smart_type(slv, slv, REUSABLE_RESOURCE_NAME)
{
    new_fields(*this, {new field(slv.get_type("real"), REUSABLE_RESOURCE_CAPACITY)}); // we add the 'capacity' field..
    new_constructors({new rr_constructor(*this)});                                    // we add a constructor..
    new_predicates({new use_predicate(*this)}, false);                                // we add the 'Use' predicate, without notifying neither the resource nor its supertypes..
}
reusable_resource::~reusable_resource() {}

std::vector<std::pair<lit, double>> reusable_resource::get_current_incs()
{
    std::vector<std::pair<lit, double>> incs;
    // TODO: add code for finding inconsistencies here..
    return incs;
}

void reusable_resource::new_predicate(predicate &) { throw std::logic_error("it is not possible to define predicates on a reusable resource.."); }

void reusable_resource::new_fact(atom_flaw &f)
{
    // we apply interval-predicate whenever the fact becomes active..
    atom &atm = f.get_atom();
    set_ni(atm.get_sigma());
    static_cast<predicate &>(get_predicate(REUSABLE_RESOURCE_USE_PREDICATE_NAME)).apply_rule(atm);
    restore_ni();

    // we avoid unification..
    if (!get_core().get_sat_core().new_clause({lit(f.get_phi(), false), atm.get_sigma()}))
        throw std::runtime_error("the problem is inconsistent..");

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

reusable_resource::use_predicate::use_predicate(reusable_resource &rr) : predicate(rr.get_core(), rr, REUSABLE_RESOURCE_USE_PREDICATE_NAME, {new field(rr.get_core().get_type(REAL_KEYWORD), REUSABLE_RESOURCE_USE_AMOUNT_NAME), new field(rr, TAU)}, {static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({REUSABLE_RESOURCE_USE_AMOUNT_NAME})), static_cast<const ast::expression *>(new ast::real_literal_expression(0))))))}) { new_supertypes({&rr.get_core().get_predicate("IntervalPredicate")}); }
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
} // namespace ratio