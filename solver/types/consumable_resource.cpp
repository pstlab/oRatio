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
        new_predicates({p_pred, c_pred}, false);                                                                                                                             // we add the 'Produce' and the 'Consume' predicates, without notifying neither the resource nor its supertypes..
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

    void consumable_resource::new_predicate(predicate &pred) noexcept
    {
        assert(get_solver().is_interval(pred));
        assert(c_pred->is_assignable_from(pred) || p_pred->is_assignable_from(pred));
        // each consumable-resource predicate has a tau parameter indicating on which resource the atoms insist on..
        new_fields(pred, {new field(static_cast<type &>(pred.get_scope()), TAU)});
    }
    void consumable_resource::new_atom(atom_flaw &f)
    {
        atom &atm = f.get_atom();
        if (f.is_fact)
        { // we apply produce-predicate or the consume-predicate whenever the fact becomes active..
            set_ni(lit(atm.get_sigma()));
            if (p_pred->is_assignable_from(atm.get_type()))
                p_pred->apply_rule(atm);
            else
                c_pred->apply_rule(atm);
            restore_ni();
        }

        // we avoid unification..
        if (!get_core().get_sat_core().new_clause({!f.get_phi(), lit(atm.get_sigma())}))
            throw unsolvable_exception();

        // since flaws might require planning, we can't store the variables for on-line flaw resolution..

        // we store, for the atom, its atom listener..
        atoms.emplace_back(&atm, new cr_atom_listener(*this, atm));

        // we filter out those atoms which are not strictly active..
        if (atm.get_core().get_sat_core().value(atm.get_sigma()) == True)
        {
            expr c_scope = atm.get(TAU);
            if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))                  // the 'tau' parameter is a variable..
                for (const auto &val : atm.get_core().get_ov_theory().value(enum_scope->ev)) // we check for all its allowed values..
                    to_check.insert(static_cast<const item *>(val));
            else // the 'tau' parameter is a constant..
                to_check.insert(&*c_scope);
        }
    }

    consumable_resource::cr_constructor::cr_constructor(consumable_resource &cr) : constructor(cr, {new field(cr.get_core().get_type(REAL_KEYWORD), CONSUMABLE_RESOURCE_INITIAL_AMOUNT), new field(cr.get_core().get_type(REAL_KEYWORD), CONSUMABLE_RESOURCE_CAPACITY)}, {{CONSUMABLE_RESOURCE_INITIAL_AMOUNT, {static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, CONSUMABLE_RESOURCE_INITIAL_AMOUNT)}))}}, {CONSUMABLE_RESOURCE_CAPACITY, {static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, CONSUMABLE_RESOURCE_CAPACITY)}))}}}, {static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, CONSUMABLE_RESOURCE_INITIAL_AMOUNT)})), static_cast<const ast::expression *>(new ast::real_literal_expression(riddle::real_token(0, 0, 0, 0, rational::ZERO))))))), static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, CONSUMABLE_RESOURCE_CAPACITY)})), static_cast<const ast::expression *>(new ast::real_literal_expression(riddle::real_token(0, 0, 0, 0, rational::ZERO)))))))}) {}

    consumable_resource::produce_predicate::produce_predicate(consumable_resource &cr) : predicate(cr, CONSUMABLE_RESOURCE_PRODUCE_PREDICATE_NAME, {new field(cr.get_core().get_type(REAL_KEYWORD), CONSUMABLE_RESOURCE_USE_AMOUNT_NAME)}, {static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, CONSUMABLE_RESOURCE_USE_AMOUNT_NAME)})), static_cast<const ast::expression *>(new ast::real_literal_expression(riddle::real_token(0, 0, 0, 0, rational::ZERO)))))))}) { new_supertypes({&cr.get_core().get_predicate(RATIO_INTERVAL)}); }

    consumable_resource::consume_predicate::consume_predicate(consumable_resource &cr) : predicate(cr, CONSUMABLE_RESOURCE_CONSUME_PREDICATE_NAME, {new field(cr.get_core().get_type(REAL_KEYWORD), CONSUMABLE_RESOURCE_USE_AMOUNT_NAME)}, {static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, CONSUMABLE_RESOURCE_USE_AMOUNT_NAME)})), static_cast<const ast::expression *>(new ast::real_literal_expression(riddle::real_token(0, 0, 0, 0, rational::ZERO)))))))}) { new_supertypes({&cr.get_core().get_predicate(RATIO_INTERVAL)}); }

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
        // we partition atoms for each consumable-resource they might insist on..
        std::unordered_map<const item *, std::vector<atom *>> cr_instances;
        for ([[maybe_unused]] const auto &[atm, atm_lstnr] : atoms)
            if (get_core().get_sat_core().value(atm->get_sigma()) == True) // we filter out those which are not strictly active..
            {
                expr c_scope = atm->get(TAU);
                if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))
                {
                    for (const auto &val : get_core().get_ov_theory().value(enum_scope->ev))
                        cr_instances[static_cast<const item *>(val)].emplace_back(atm);
                }
                else
                    cr_instances[static_cast<item *>(&*c_scope)].emplace_back(atm);
            }

        for (const auto &[cr, atms] : cr_instances)
        {
            json tl;
            tl->set("id", new long_val(cr->get_id()));
#if defined(VERBOSE_LOG) || defined(BUILD_LISTENERS)
            tl->set("name", new string_val(get_core().guess_name(*cr)));
#endif
            tl->set("type", new string_val(CONSUMABLE_RESOURCE_NAME));

            arith_expr initial_amount = cr->get_exprs().at(CONSUMABLE_RESOURCE_INITIAL_AMOUNT);
            inf_rational c_initial_amount = get_core().arith_value(initial_amount);
            tl->set("initial_amount", to_json(c_initial_amount));

            arith_expr capacity = cr->get_exprs().at(CONSUMABLE_RESOURCE_CAPACITY);
            inf_rational c_capacity = get_core().arith_value(capacity);
            tl->set("capacity", to_json(c_capacity));

            // for each pulse, the atoms starting at that pulse..
            std::map<inf_rational, std::set<atom *>> starting_atoms;
            // for each pulse, the atoms ending at that pulse..
            std::map<inf_rational, std::set<atom *>> ending_atoms;
            // all the pulses of the timeline..
            std::set<inf_rational> pulses;

            for (const auto &atm : atms)
            {
                arith_expr s_expr = atm->get(RATIO_START);
                arith_expr e_expr = atm->get(RATIO_END);
                inf_rational start = get_core().arith_value(s_expr);
                inf_rational end = get_core().arith_value(e_expr);
                starting_atoms[start].insert(atm);
                ending_atoms[end].insert(atm);
                pulses.insert(start);
                pulses.insert(end);
            }
            arith_expr origin_expr = get_core().get("origin");
            arith_expr horizon_expr = get_core().get("horizon");
            pulses.insert(get_core().arith_value(origin_expr));
            pulses.insert(get_core().arith_value(horizon_expr));

            std::set<atom *> overlapping_atoms;
            std::set<inf_rational>::iterator p = pulses.begin();
            if (const auto at_start_p = starting_atoms.find(*p); at_start_p != starting_atoms.cend())
                overlapping_atoms.insert(at_start_p->second.cbegin(), at_start_p->second.cend());
            if (const auto at_end_p = ending_atoms.find(*p); at_end_p != ending_atoms.cend())
                for (const auto &a : at_end_p->second)
                    overlapping_atoms.erase(a);

            std::vector<json> j_vals;
            inf_rational c_val = c_initial_amount;
            for (p = std::next(p); p != pulses.end(); ++p)
            {
                json j_val;
                j_val->set("from", to_json(*std::prev(p)));
                j_val->set("to", to_json(*p));

                std::vector<json> j_atms;
                inf_rational c_angular_coefficient; // the concurrent resource update..
                for (const auto &atm : overlapping_atoms)
                {
                    arith_expr amount = atm->get(CONSUMABLE_RESOURCE_USE_AMOUNT_NAME);
                    inf_rational c_coeff;
                    if (p_pred->is_assignable_from(atm->get_type()))
                        c_coeff = get_core().arith_value(amount);
                    else
                        c_coeff = -get_core().arith_value(amount);
                    arith_expr s_expr = atm->get(RATIO_START);
                    arith_expr e_expr = atm->get(RATIO_END);
                    c_coeff /= (get_core().arith_value(e_expr) - get_core().arith_value(s_expr)).get_rational();
                    c_angular_coefficient += c_coeff;

                    j_atms.emplace_back(new long_val(atm->get_id()));
                }
                j_val->set("atoms", new array_val(j_atms));

                j_val->set("start", to_json(c_val));
                c_val += (c_angular_coefficient *= (*p - *std::prev(p)).get_rational());
                j_val->set("end", to_json(c_val));

                j_vals.emplace_back(j_val);

                if (const auto at_start_p = starting_atoms.find(*p); at_start_p != starting_atoms.cend())
                    overlapping_atoms.insert(at_start_p->second.cbegin(), at_start_p->second.cend());
                if (const auto at_end_p = ending_atoms.find(*p); at_end_p != ending_atoms.cend())
                    for (const auto &a : at_end_p->second)
                        overlapping_atoms.erase(a);
            }
            tl->set("values", new array_val(j_vals));

            tls.emplace_back(tl);
        }

        return new array_val(tls);
    }
} // namespace ratio
