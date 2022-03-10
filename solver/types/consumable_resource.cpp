#include "consumable_resource.h"
#include "solver.h"
#include "atom.h"
#include "field.h"
#include "atom_flaw.h"
#include "combinations.h"
#include "core_parser.h"
#include <stdexcept>
#include <list>

using namespace smt;

namespace ratio
{
    consumable_resource::consumable_resource(solver &slv) : smart_type(slv, slv, CONSUMABLE_RESOURCE_NAME), timelines_extractor()
    {
        new_fields(*this, {new field(slv.get_type(REAL_KEYWORD), CONSUMABLE_RESOURCE_INITIAL_AMOUNT), new field(slv.get_type(REAL_KEYWORD), CONSUMABLE_RESOURCE_CAPACITY)}); // we add the 'initial_amount' and the 'capacity' fields..
        new_constructors({new cr_constructor(*this)});                                                                                                                       // we add a constructor..
        new_predicates({new produce_predicate(*this), new consume_predicate(*this)}, false);                                                                                 // we add the 'Produce' and the 'Consume' predicates, without notifying neither the resource nor its supertypes..
    }
    consumable_resource::~consumable_resource()
    {
        // we clear the atom listeners..
        for ([[maybe_unused]] const auto &[atm, lstnrs] : atoms)
            delete lstnrs;
    }

    std::vector<std::vector<std::pair<lit, double>>> consumable_resource::get_current_incs()
    {
        std::vector<std::vector<std::pair<lit, double>>> incs;
        return incs;
    }

    consumable_resource::cr_constructor::cr_constructor(consumable_resource &cr) : constructor(cr.get_core(), cr, {new field(cr.get_core().get_type(REAL_KEYWORD), CONSUMABLE_RESOURCE_INITIAL_AMOUNT), new field(cr.get_core().get_type(REAL_KEYWORD), CONSUMABLE_RESOURCE_INITIAL_AMOUNT)}, {{CONSUMABLE_RESOURCE_INITIAL_AMOUNT, {static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, CONSUMABLE_RESOURCE_INITIAL_AMOUNT)}))}}, {CONSUMABLE_RESOURCE_CAPACITY, {static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, CONSUMABLE_RESOURCE_CAPACITY)}))}}}, {static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, CONSUMABLE_RESOURCE_INITIAL_AMOUNT)})), static_cast<const ast::expression *>(new ast::real_literal_expression(riddle::real_token(0, 0, 0, 0, rational::ZERO))))))), static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, CONSUMABLE_RESOURCE_CAPACITY)})), static_cast<const ast::expression *>(new ast::real_literal_expression(riddle::real_token(0, 0, 0, 0, rational::ZERO)))))))}) {}

    consumable_resource::produce_predicate::produce_predicate(consumable_resource &cr) : predicate(cr.get_core(), cr, CONSUMABLE_RESOURCE_PRODUCE_PREDICATE_NAME, {new field(cr.get_core().get_type(REAL_KEYWORD), CONSUMABLE_RESOURCE_USE_AMOUNT_NAME)}, {static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, CONSUMABLE_RESOURCE_USE_AMOUNT_NAME)})), static_cast<const ast::expression *>(new ast::real_literal_expression(riddle::real_token(0, 0, 0, 0, rational::ZERO)))))))}) { new_supertypes({&cr.get_core().get_predicate(RATIO_INTERVAL)}); }

    consumable_resource::consume_predicate::consume_predicate(consumable_resource &cr) : predicate(cr.get_core(), cr, CONSUMABLE_RESOURCE_CONSUME_PREDICATE_NAME, {new field(cr.get_core().get_type(REAL_KEYWORD), CONSUMABLE_RESOURCE_USE_AMOUNT_NAME)}, {static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, CONSUMABLE_RESOURCE_USE_AMOUNT_NAME)})), static_cast<const ast::expression *>(new ast::real_literal_expression(riddle::real_token(0, 0, 0, 0, rational::ZERO)))))))}) { new_supertypes({&cr.get_core().get_predicate(RATIO_INTERVAL)}); }

    consumable_resource::cr_atom_listener::cr_atom_listener(consumable_resource &cr, atom &atm) : atom_listener(atm), cr(cr) {}

    void consumable_resource::cr_atom_listener::something_changed()
    {
        // we filter out those atoms which are not strictly active..
        if (atm.get_core().get_sat_core().value(atm.get_sigma()) == True)
        {
            expr c_scope = atm.get(TAU);
            if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))                  // the 'tau' parameter is a variable..
                for (const auto &val : atm.get_core().get_ov_theory().value(enum_scope->ev)) // we check for all its allowed values..
                    cr.to_check.insert(static_cast<const item *>(val));
            else // the 'tau' parameter is a constant..
                cr.to_check.insert(&*c_scope);
        }
    }

    json consumable_resource::extract_timelines() const noexcept
    {
        std::vector<json> tls;
        return new array_val(tls);
    }
} // namespace ratio
