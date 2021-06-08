#include "core_parser.h"
#include "core.h"
#include "field.h"
#include "atom.h"
#include "predicate.h"
#include "typedef_type.h"
#include "enum_type.h"
#include "constructor.h"
#include "method.h"
#include "conjunction.h"

namespace ratio
{
    namespace ast
    {
        expression::expression() {}
        expression::~expression() {}

        CORE_EXPORT bool_literal_expression::bool_literal_expression(const riddle::bool_token &l) : riddle::ast::bool_literal_expression(l) {}
        CORE_EXPORT bool_literal_expression::~bool_literal_expression() {}
        expr bool_literal_expression::evaluate(const scope &scp, context &) const { return scp.get_core().new_bool(literal.val); }

        CORE_EXPORT int_literal_expression::int_literal_expression(const riddle::int_token &l) : riddle::ast::int_literal_expression(l) {}
        CORE_EXPORT int_literal_expression::~int_literal_expression() {}
        expr int_literal_expression::evaluate(const scope &scp, context &) const { return scp.get_core().new_int(literal.val); }

        CORE_EXPORT real_literal_expression::real_literal_expression(const riddle::real_token &l) : riddle::ast::real_literal_expression(l) {}
        CORE_EXPORT real_literal_expression::~real_literal_expression() {}
        expr real_literal_expression::evaluate(const scope &scp, context &) const { return scp.get_core().new_real(literal.val); }

        CORE_EXPORT string_literal_expression::string_literal_expression(const riddle::string_token &l) : riddle::ast::string_literal_expression(l) {}
        CORE_EXPORT string_literal_expression::~string_literal_expression() {}
        expr string_literal_expression::evaluate(const scope &scp, context &) const { return scp.get_core().new_string(literal.str); }

        cast_expression::cast_expression(const std::vector<riddle::id_token> &tp, const riddle::ast::expression *const e) : riddle::ast::cast_expression(tp, e) {}
        cast_expression::~cast_expression() {}
        expr cast_expression::evaluate(const scope &scp, context &ctx) const { return dynamic_cast<const ast::expression *>(xpr)->evaluate(scp, ctx); }

        plus_expression::plus_expression(const riddle::ast::expression *const e) : riddle::ast::plus_expression(e) {}
        plus_expression::~plus_expression() {}
        expr plus_expression::evaluate(const scope &scp, context &ctx) const { return dynamic_cast<const ast::expression *>(xpr)->evaluate(scp, ctx); }

        minus_expression::minus_expression(const riddle::ast::expression *const e) : riddle::ast::minus_expression(e) {}
        minus_expression::~minus_expression() {}
        expr minus_expression::evaluate(const scope &scp, context &ctx) const { return scp.get_core().minus(dynamic_cast<const ast::expression *const>(xpr)->evaluate(scp, ctx)); }

        not_expression::not_expression(const riddle::ast::expression *const e) : riddle::ast::not_expression(e) {}
        not_expression::~not_expression() {}
        expr not_expression::evaluate(const scope &scp, context &ctx) const { return scp.get_core().negate(dynamic_cast<const ast::expression *const>(xpr)->evaluate(scp, ctx)); }

        constructor_expression::constructor_expression(const std::vector<riddle::id_token> &it, const std::vector<const riddle::ast::expression *> &es) : riddle::ast::constructor_expression(it, es) {}
        constructor_expression::~constructor_expression() {}
        expr constructor_expression::evaluate(const scope &scp, context &ctx) const
        {
            scope *s = const_cast<scope *>(&scp);
            for (const auto &tp : instance_type)
                s = &s->get_type(tp.id);

            std::vector<expr> exprs;
            std::vector<const type *> par_types;
            for (const auto &ex : expressions)
            {
                expr i = dynamic_cast<const ast::expression *>(ex)->evaluate(scp, ctx);
                exprs.push_back(i);
                par_types.push_back(&i->get_type());
            }

            return static_cast<type *>(s)->get_constructor(par_types).new_instance(ctx, exprs);
        }

        eq_expression::eq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::eq_expression(l, r) {}
        eq_expression::~eq_expression() {}
        expr eq_expression::evaluate(const scope &scp, context &ctx) const
        {
            expr l = dynamic_cast<const ast::expression *const>(left)->evaluate(scp, ctx);
            expr r = dynamic_cast<const ast::expression *const>(right)->evaluate(scp, ctx);
            return scp.get_core().eq(l, r);
        }

        neq_expression::neq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::neq_expression(l, r) {}
        neq_expression::~neq_expression() {}
        expr neq_expression::evaluate(const scope &scp, context &ctx) const
        {
            expr l = dynamic_cast<const ast::expression *const>(left)->evaluate(scp, ctx);
            expr r = dynamic_cast<const ast::expression *const>(right)->evaluate(scp, ctx);
            return scp.get_core().negate(scp.get_core().eq(l, r));
        }

        CORE_EXPORT lt_expression::lt_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::lt_expression(l, r) {}
        CORE_EXPORT lt_expression::~lt_expression() {}
        expr lt_expression::evaluate(const scope &scp, context &ctx) const
        {
            arith_expr l = dynamic_cast<const ast::expression *const>(left)->evaluate(scp, ctx);
            arith_expr r = dynamic_cast<const ast::expression *const>(right)->evaluate(scp, ctx);
            return scp.get_core().lt(l, r);
        }

        CORE_EXPORT leq_expression::leq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::leq_expression(l, r) {}
        CORE_EXPORT leq_expression::~leq_expression() {}
        expr leq_expression::evaluate(const scope &scp, context &ctx) const
        {
            arith_expr l = dynamic_cast<const ast::expression *const>(left)->evaluate(scp, ctx);
            arith_expr r = dynamic_cast<const ast::expression *const>(right)->evaluate(scp, ctx);
            return scp.get_core().leq(l, r);
        }

        CORE_EXPORT geq_expression::geq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::geq_expression(l, r) {}
        CORE_EXPORT geq_expression::~geq_expression() {}
        expr geq_expression::evaluate(const scope &scp, context &ctx) const
        {
            arith_expr l = dynamic_cast<const ast::expression *const>(left)->evaluate(scp, ctx);
            arith_expr r = dynamic_cast<const ast::expression *const>(right)->evaluate(scp, ctx);
            return scp.get_core().geq(l, r);
        }

        CORE_EXPORT gt_expression::gt_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::gt_expression(l, r) {}
        CORE_EXPORT gt_expression::~gt_expression() {}
        expr gt_expression::evaluate(const scope &scp, context &ctx) const
        {
            arith_expr l = dynamic_cast<const ast::expression *const>(left)->evaluate(scp, ctx);
            arith_expr r = dynamic_cast<const ast::expression *const>(right)->evaluate(scp, ctx);
            return scp.get_core().gt(l, r);
        }

        function_expression::function_expression(const std::vector<riddle::id_token> &is, const riddle::id_token &fn, const std::vector<const riddle::ast::expression *> &es) : riddle::ast::function_expression(is, fn, es) {}
        function_expression::~function_expression() {}
        expr function_expression::evaluate(const scope &scp, context &ctx) const
        {
            scope *s = const_cast<scope *>(&scp);
            for (const auto &id_tk : ids)
                s = &s->get_type(id_tk.id);

            std::vector<expr> exprs;
            std::vector<const type *> par_types;
            for (const auto &ex : expressions)
            {
                expr i = dynamic_cast<const ast::expression *>(ex)->evaluate(scp, ctx);
                exprs.push_back(i);
                par_types.push_back(&i->get_type());
            }

            if (const method &m = s->get_method(function_name.id, par_types); m.get_return_type())
            {
                if (m.get_return_type() == &scp.get_core().get_type(BOOL_KEYWORD))
                    return bool_expr(static_cast<bool_item *>(m.invoke(ctx, exprs)));
                else if (m.get_return_type() == &scp.get_core().get_type(INT_KEYWORD) || m.get_return_type() == &scp.get_core().get_type(REAL_KEYWORD) || m.get_return_type() == &scp.get_core().get_type(TP_KEYWORD))
                    return arith_expr(static_cast<arith_item *>(m.invoke(ctx, exprs)));
                else
                    return expr(m.invoke(ctx, exprs));
            }
            else
                return scp.get_core().new_bool(true);
        }

        CORE_EXPORT id_expression::id_expression(const std::vector<riddle::id_token> &is) : riddle::ast::id_expression(is) {}
        CORE_EXPORT id_expression::~id_expression() {}
        expr id_expression::evaluate(const scope &, context &ctx) const
        {
            env *c_e = &*ctx;
            for (const auto &id_tk : ids)
                c_e = &*c_e->get(id_tk.id);
            return expr(static_cast<item *>(c_e));
        }

        implication_expression::implication_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::implication_expression(l, r) {}
        implication_expression::~implication_expression() {}
        expr implication_expression::evaluate(const scope &scp, context &ctx) const
        {
            bool_expr l = dynamic_cast<const ast::expression *>(left)->evaluate(scp, ctx);
            bool_expr r = dynamic_cast<const ast::expression *>(right)->evaluate(scp, ctx);
            return scp.get_core().disj({scp.get_core().negate(l), r});
        }

        disjunction_expression::disjunction_expression(const std::vector<const riddle::ast::expression *> &es) : riddle::ast::disjunction_expression(es) {}
        disjunction_expression::~disjunction_expression() {}
        expr disjunction_expression::evaluate(const scope &scp, context &ctx) const
        {
            std::vector<bool_expr> exprs;
            for (const auto &e : expressions)
                exprs.push_back(dynamic_cast<const ast::expression *>(e)->evaluate(scp, ctx));
            return scp.get_core().disj(exprs);
        }

        conjunction_expression::conjunction_expression(const std::vector<const riddle::ast::expression *> &es) : riddle::ast::conjunction_expression(es) {}
        conjunction_expression::~conjunction_expression() {}
        expr conjunction_expression::evaluate(const scope &scp, context &ctx) const
        {
            std::vector<bool_expr> exprs;
            for (const auto &e : expressions)
                exprs.push_back(dynamic_cast<const ast::expression *>(e)->evaluate(scp, ctx));
            return scp.get_core().conj(exprs);
        }

        exct_one_expression::exct_one_expression(const std::vector<const riddle::ast::expression *> &es) : riddle::ast::exct_one_expression(es) {}
        exct_one_expression::~exct_one_expression() {}
        expr exct_one_expression::evaluate(const scope &scp, context &ctx) const
        {
            std::vector<bool_expr> exprs;
            for (const auto &e : expressions)
                exprs.push_back(dynamic_cast<const ast::expression *>(e)->evaluate(scp, ctx));
            return scp.get_core().exct_one(exprs);
        }

        addition_expression::addition_expression(const std::vector<const riddle::ast::expression *> &es) : riddle::ast::addition_expression(es) {}
        addition_expression::~addition_expression() {}
        expr addition_expression::evaluate(const scope &scp, context &ctx) const
        {
            std::vector<arith_expr> exprs;
            for (const auto &e : expressions)
                exprs.push_back(dynamic_cast<const ast::expression *>(e)->evaluate(scp, ctx));
            return scp.get_core().add(exprs);
        }

        subtraction_expression::subtraction_expression(const std::vector<const riddle::ast::expression *> &es) : riddle::ast::subtraction_expression(es) {}
        subtraction_expression::~subtraction_expression() {}
        expr subtraction_expression::evaluate(const scope &scp, context &ctx) const
        {
            std::vector<arith_expr> exprs;
            for (const auto &e : expressions)
                exprs.push_back(dynamic_cast<const ast::expression *>(e)->evaluate(scp, ctx));
            return scp.get_core().sub(exprs);
        }

        multiplication_expression::multiplication_expression(const std::vector<const riddle::ast::expression *> &es) : riddle::ast::multiplication_expression(es) {}
        multiplication_expression::~multiplication_expression() {}
        expr multiplication_expression::evaluate(const scope &scp, context &ctx) const
        {
            std::vector<arith_expr> exprs;
            for (const auto &e : expressions)
                exprs.push_back(dynamic_cast<const ast::expression *>(e)->evaluate(scp, ctx));
            return scp.get_core().mult(exprs);
        }

        division_expression::division_expression(const std::vector<const riddle::ast::expression *> &es) : riddle::ast::division_expression(es) {}
        division_expression::~division_expression() {}
        expr division_expression::evaluate(const scope &scp, context &ctx) const
        {
            std::vector<arith_expr> exprs;
            for (const auto &e : expressions)
                exprs.push_back(dynamic_cast<const ast::expression *>(e)->evaluate(scp, ctx));
            return scp.get_core().div(exprs);
        }

        statement::statement() {}
        statement::~statement() {}

        local_field_statement::local_field_statement(const std::vector<riddle::id_token> &ft, const std::vector<riddle::id_token> &ns, const std::vector<const riddle::ast::expression *> &es) : riddle::ast::local_field_statement(ft, ns, es) {}
        local_field_statement::~local_field_statement() {}
        void local_field_statement::execute(const scope &scp, context &ctx) const
        {
            for (size_t i = 0; i < names.size(); ++i)
            {
                if (xprs[i])
                    ctx->exprs.emplace(names[i].id, dynamic_cast<const ast::expression *>(xprs[i])->evaluate(scp, ctx));
                else
                {
                    scope *s = const_cast<scope *>(&scp);
                    for (const auto &tp : field_type)
                        s = &s->get_type(tp.id);
                    type *t = static_cast<type *>(s);
                    if (t->is_primitive())
                        ctx->exprs.emplace(names[i].id, t->new_instance(ctx));
                    else if (!t->get_instances().empty())
                        ctx->exprs.emplace(names[i].id, t->new_existential());
                    else
                        throw inconsistency_exception();
                }

                if (&scp == &scp.get_core()) // we create fields for root items..
                    scp.get_core().fields.emplace(names[i].id, new field(ctx->exprs.at(names[i].id)->get_type(), names[i].id));
            }
        }

        assignment_statement::assignment_statement(const std::vector<riddle::id_token> &is, const riddle::id_token &i, const riddle::ast::expression *const e) : riddle::ast::assignment_statement(is, i, e) {}
        assignment_statement::~assignment_statement() {}
        void assignment_statement::execute(const scope &scp, context &ctx) const
        {
            env *c_e = &*ctx;
            for (const auto &c_id : ids)
                c_e = &*c_e->get(c_id.id);
            c_e->exprs.emplace(id.id, dynamic_cast<const ast::expression *>(xpr)->evaluate(scp, ctx));
        }

        CORE_EXPORT expression_statement::expression_statement(const riddle::ast::expression *const e) : riddle::ast::expression_statement(e) {}
        CORE_EXPORT expression_statement::~expression_statement() {}
        void expression_statement::execute(const scope &scp, context &ctx) const
        {
            bool_expr be = dynamic_cast<const ast::expression *const>(xpr)->evaluate(scp, ctx);
            scp.get_core().assert_facts({be->l});
        }

        disjunction_statement::disjunction_statement(const std::vector<std::pair<const std::vector<const riddle::ast::statement *>, const riddle::ast::expression *const>> &conjs) : riddle::ast::disjunction_statement(conjs) {}
        disjunction_statement::~disjunction_statement() {}
        void disjunction_statement::execute(const scope &scp, context &ctx) const
        {
            std::vector<const conjunction *> cs;
            for (const auto &[stmnts, xpr] : conjunctions)
            {
                smt::rational cost(1);
                if (xpr)
                {
                    arith_expr a_xpr = dynamic_cast<const ast::expression *const>(xpr)->evaluate(scp, ctx);
                    if (!a_xpr->l.vars.empty())
                        throw std::invalid_argument("invalid disjunct cost: expected a constant..");
                    cost = scp.get_core().arith_value(a_xpr).get_rational();
                }
                cs.push_back(new conjunction(scp.get_core(), const_cast<scope &>(scp), cost, stmnts));
            }
            scp.get_core().new_disjunction(ctx, cs);
        }

        conjunction_statement::conjunction_statement(const std::vector<const riddle::ast::statement *> &stmnts) : riddle::ast::conjunction_statement(stmnts) {}
        conjunction_statement::~conjunction_statement() {}
        void conjunction_statement::execute(const scope &scp, context &ctx) const
        {
            for (const auto &st : statements)
                dynamic_cast<const ast::statement *>(st)->execute(scp, ctx);
        }

        formula_statement::formula_statement(const bool &isf, const riddle::id_token &fn, const std::vector<riddle::id_token> &scp, const riddle::id_token &pn, const std::vector<std::pair<const riddle::id_token, const riddle::ast::expression *const>> &assns) : riddle::ast::formula_statement(isf, fn, scp, pn, assns) {}
        formula_statement::~formula_statement() {}
        void formula_statement::execute(const scope &scp, context &ctx) const
        {
            predicate *p = nullptr;
            std::unordered_map<std::string, expr> assgnments;
            if (!formula_scope.empty()) // the scope is explicitely declared..
            {
                env *c_scope = &*ctx;
                for (const auto &s : formula_scope)
                    c_scope = &*c_scope->get(s.id);
                p = &static_cast<item *>(c_scope)->get_type().get_predicate(predicate_name.id);

                if (var_item *ee = dynamic_cast<var_item *>(c_scope)) // the scope is an enumerative expression..
                    assgnments.emplace(TAU, ee);
                else // the scope is a single item..
                    assgnments.emplace(TAU, context(c_scope));
            }
            else
            {
                p = &scp.get_predicate(predicate_name.id);
                if (&p->get_scope() != &scp.get_core()) // we inherit the scope..
                    assgnments.emplace(TAU, ctx->get(TAU));
            }

            for (const auto &[id_tkn, xpr] : assignments)
            {
                expr e = dynamic_cast<const ast::expression *>(xpr)->evaluate(scp, ctx);
                const type &tt = p->get_field(id_tkn.id).get_type(); // the target type..
                if (tt.is_assignable_from(e->get_type()))            // the target type is a superclass of the assignment..
                    assgnments.emplace(id_tkn.id, e);
                else if (e->get_type().is_assignable_from(tt))        // the target type is a subclass of the assignment..
                    if (var_item *ae = dynamic_cast<var_item *>(&*e)) // some of the allowed values might be inhibited..
                    {
                        std::unordered_set<const smt::var_value *> alwd_vals = scp.get_core().get_ov_theory().value(ae->ev); // the allowed values..
                        std::vector<smt::lit> not_alwd_vals;                                                                 // the not allowed values..
                        for (const auto &ev : alwd_vals)
                            if (!tt.is_assignable_from(static_cast<const item *>(ev)->get_type())) // the target type is not a superclass of the value..
                                not_alwd_vals.push_back(!scp.get_core().get_ov_theory().allows(ae->ev, *ev));
                        if (alwd_vals.size() == not_alwd_vals.size()) // none of the values is allowed..
                            throw inconsistency_exception();
                        else // we inhibit the not allowed values..
                            scp.get_core().assert_facts(not_alwd_vals);
                    }
                    else // the evaluated expression is a constant which cannot be assigned to the target type (which is a subclass of the type of the evaluated expression)..
                        throw inconsistency_exception();
                else // the evaluated expression is unrelated with the target type (we are probably in the presence of a modeling error!)..
                    throw inconsistency_exception();
            }

            atom *a;
            if (!assgnments.count(TAU))
            { // the new atom's scope is the core..
                context c_scope(&scp.get_core());
                a = static_cast<atom *>(&*p->new_instance(c_scope));
            }
            else
            { // we have computed the new atom's scope above..
                context c_scope(assgnments.at(TAU));
                a = static_cast<atom *>(&*p->new_instance(c_scope));
            }

            // we assign fields..
            a->exprs.insert(assgnments.cbegin(), assgnments.cend());

            // we initialize atom's fields..
            std::queue<predicate *> q;
            q.push(p);
            while (!q.empty())
            {
                for (const auto &arg : q.front()->get_args())
                    if (!a->exprs.count(arg->get_name()))
                    { // the field is uninstantiated..
                        type &tp = const_cast<type &>(arg->get_type());
                        if (tp.is_primitive())
                            a->exprs.emplace(arg->get_name(), tp.new_instance(ctx));
                        else
                            a->exprs.emplace(arg->get_name(), tp.new_existential());
                    }
                for (const auto &sp : q.front()->get_supertypes())
                    q.push(static_cast<predicate *>(sp));
                q.pop();
            }

            scp.get_core().new_atom(*a, is_fact);
            ctx->exprs.emplace(formula_name.id, expr(a));
        }

        return_statement::return_statement(const riddle::ast::expression *const e) : riddle::ast::return_statement(e) {}
        return_statement::~return_statement() {}
        void return_statement::execute(const scope &scp, context &ctx) const { ctx->exprs.emplace(RETURN_KEYWORD, dynamic_cast<const ast::expression *>(xpr)->evaluate(scp, ctx)); }

        type_declaration::type_declaration() {}
        type_declaration::~type_declaration() {}

        method_declaration::method_declaration(const std::vector<riddle::id_token> &rt, const riddle::id_token &n, const std::vector<std::pair<const std::vector<riddle::id_token>, const riddle::id_token>> &pars, const std::vector<const riddle::ast::statement *> &stmnts) : riddle::ast::method_declaration(rt, n, pars, stmnts) {}
        method_declaration::~method_declaration() {}
        void method_declaration::refine(scope &scp) const
        {
            type *rt = nullptr;
            if (!return_type.empty())
            {
                scope *s = &scp;
                for (const auto &id_tk : return_type)
                    s = &s->get_type(id_tk.id);
                rt = static_cast<type *>(s);
            }

            std::vector<const field *> args;
            for (const auto &[id_tkns, id_tkn] : parameters)
            {
                scope *s = &scp;
                for (const auto &id_tk : id_tkns)
                    s = &s->get_type(id_tk.id);
                type *tp = static_cast<type *>(s);
                args.push_back(new field(*tp, id_tkn.id));
            }

            if (method *m = new method(scp.get_core(), scp, rt, name.id, args, statements); core *c = dynamic_cast<core *>(&scp))
                c->new_methods({m});
            else if (type *t = dynamic_cast<type *>(&scp))
                t->new_methods({m});
        }

        predicate_declaration::predicate_declaration(const riddle::id_token &n, const std::vector<std::pair<const std::vector<riddle::id_token>, const riddle::id_token>> &pars, const std::vector<std::vector<riddle::id_token>> &pl, const std::vector<const riddle::ast::statement *> &stmnts) : riddle::ast::predicate_declaration(n, pars, pl, stmnts) {}
        predicate_declaration::~predicate_declaration() {}
        void predicate_declaration::refine(scope &scp) const
        {
            std::vector<const field *> args;
            for (const auto &[id_tkns, id_tkn] : parameters)
            {
                scope *s = &scp;
                for (const auto &id_tk : id_tkns)
                    s = &s->get_type(id_tk.id);
                type *tp = static_cast<type *>(s);
                args.push_back(new field(*tp, id_tkn.id));
            }

            predicate *p = new predicate(scp.get_core(), scp, name.id, args, statements);

            // we add the supertypes.. notice that we do not support forward declaration for predicate supertypes!!
            for (const auto &sp : predicate_list)
            {
                scope *s = &scp;
                for (const auto &id_tk : sp)
                    s = &s->get_predicate(id_tk.id);
                p->new_supertypes({static_cast<predicate *>(s)});
            }

            if (core *c = dynamic_cast<core *>(&scp))
                c->new_predicates({p});
            else if (type *t = dynamic_cast<type *>(&scp))
                t->new_predicates({p});
        }

        typedef_declaration::typedef_declaration(const riddle::id_token &n, const riddle::id_token &pt, const riddle::ast::expression *const e) : riddle::ast::typedef_declaration(n, pt, e) {}
        typedef_declaration::~typedef_declaration() {}
        void typedef_declaration::declare(scope &scp) const
        {
            // A new typedef type has been declared..
            typedef_type *td = new typedef_type(scp.get_core(), scp, name.id, scp.get_type(primitive_type.id), xpr);

            if (core *c = dynamic_cast<core *>(&scp))
                c->new_types({td});
            else if (type *t = dynamic_cast<type *>(&scp))
                t->new_types({td});
        }

        enum_declaration::enum_declaration(const riddle::id_token &n, const std::vector<riddle::string_token> &es, const std::vector<std::vector<riddle::id_token>> &trs) : riddle::ast::enum_declaration(n, es, trs) {}
        enum_declaration::~enum_declaration() {}
        void enum_declaration::declare(scope &scp) const
        {
            // A new enum type has been declared..
            enum_type *et = new enum_type(scp.get_core(), scp, name.id);

            // We add the enum values..
            for (const auto &e : enums)
                et->instances.push_back(scp.get_core().new_string(e.str));

            if (core *c = dynamic_cast<core *>(&scp))
                c->new_types({et});
            else if (type *t = dynamic_cast<type *>(&scp))
                t->new_types({et});
        }
        void enum_declaration::refine(scope &scp) const
        {
            if (!type_refs.empty())
            {
                enum_type *et = static_cast<enum_type *>(&scp.get_type(name.id));
                for (const auto &tr : type_refs)
                {
                    scope *s = &scp;
                    for (const auto &id_tk : tr)
                        s = &s->get_type(id_tk.id);
                    et->enums.push_back(static_cast<enum_type *>(s));
                }
            }
        }

        field_declaration::field_declaration(const std::vector<riddle::id_token> &tp, const std::vector<const riddle::ast::variable_declaration *> &ds) : riddle::ast::field_declaration(tp, ds) {}
        field_declaration::~field_declaration() {}
        void field_declaration::refine(scope &scp) const
        {
            // we add fields to the current scope..
            scope *s = &scp;
            for (const auto &id_tk : field_type)
                s = &s->get_type(id_tk.id);
            type *tp = static_cast<type *>(s);

            for (const auto &vd : declarations)
                scp.new_fields({new field(*tp, dynamic_cast<const ast::variable_declaration *>(vd)->name.id, dynamic_cast<const ast::variable_declaration *>(vd)->xpr)});
        }

        constructor_declaration::constructor_declaration(const std::vector<std::pair<const std::vector<riddle::id_token>, const riddle::id_token>> &pars, const std::vector<std::pair<const riddle::id_token, const std::vector<const riddle::ast::expression *>>> &il, const std::vector<const riddle::ast::statement *> &stmnts) : riddle::ast::constructor_declaration(pars, il, stmnts) {}
        constructor_declaration::~constructor_declaration() {}
        void constructor_declaration::refine(scope &scp) const
        {
            std::vector<const field *> args;
            for (const auto &[id_tkns, id_tkn] : parameters)
            {
                scope *s = &scp;
                for (const auto &id_tk : id_tkns)
                    s = &s->get_type(id_tk.id);
                type *tp = static_cast<type *>(s);
                args.push_back(new field(*tp, id_tkn.id));
            }

            std::vector<std::pair<const std::string, const std::vector<const riddle::ast::expression *>>> il;
            for (const auto &[id_tkn, xprs] : init_list)
                il.push_back({id_tkn.id, xprs});

            static_cast<type &>(scp).new_constructors({new constructor(scp.get_core(), scp, args, il, statements)});
        }

        class_declaration::class_declaration(const riddle::id_token &n, const std::vector<std::vector<riddle::id_token>> &bcs, const std::vector<const riddle::ast::field_declaration *> &fs, const std::vector<const riddle::ast::constructor_declaration *> &cs, const std::vector<const riddle::ast::method_declaration *> &ms, const std::vector<const riddle::ast::predicate_declaration *> &ps, const std::vector<const riddle::ast::type_declaration *> &ts) : riddle::ast::class_declaration(n, bcs, fs, cs, ms, ps, ts) {}
        class_declaration::~class_declaration() {}
        void class_declaration::declare(scope &scp) const
        {
            // A new type has been declared..
            type *tp = new type(scp.get_core(), scp, name.id);

            if (core *c = dynamic_cast<core *>(&scp))
                c->new_types({tp});
            else if (type *t = dynamic_cast<type *>(&scp))
                t->new_types({tp});

            for (const auto &c_tp : types)
                dynamic_cast<const ast::type_declaration *>(c_tp)->declare(*tp);
        }
        void class_declaration::refine(scope &scp) const
        {
            type &tp = scp.get_type(name.id);
            for (const auto &bc : base_classes)
            {
                scope *s = &scp;
                for (const auto &id_tk : bc)
                    s = &s->get_type(id_tk.id);
                tp.new_supertypes({static_cast<type *>(s)});
            }

            for (const auto &f : fields)
                dynamic_cast<const ast::field_declaration *>(f)->refine(tp);

            if (constructors.empty())
                tp.new_constructors({new constructor(scp.get_core(), tp, {}, {}, {})}); // we add a default constructor..
            else
                for (const auto &c : constructors)
                    dynamic_cast<const ast::constructor_declaration *>(c)->refine(tp);

            for (const auto &m : methods)
                dynamic_cast<const ast::method_declaration *>(m)->refine(tp);
            for (const auto &p : predicates)
                dynamic_cast<const ast::predicate_declaration *>(p)->refine(tp);
            for (const auto &t : types)
                dynamic_cast<const ast::type_declaration *>(t)->refine(tp);
        }

        compilation_unit::compilation_unit(const std::vector<const riddle::ast::method_declaration *> &ms, const std::vector<const riddle::ast::predicate_declaration *> &ps, const std::vector<const riddle::ast::type_declaration *> &ts, const std::vector<const riddle::ast::statement *> &stmnts) : riddle::ast::compilation_unit(ms, ps, ts, stmnts) {}
        compilation_unit::~compilation_unit() {}
        void compilation_unit::declare(scope &scp) const
        {
            for (const auto &t : types)
                dynamic_cast<const ast::type_declaration *>(t)->declare(scp);
        }
        void compilation_unit::refine(scope &scp) const
        {
            for (const auto &t : types)
                dynamic_cast<const ast::type_declaration *>(t)->refine(scp);
            for (const auto &m : methods)
                dynamic_cast<const ast::method_declaration *>(m)->refine(scp);
            for (const auto &p : predicates)
                dynamic_cast<const ast::predicate_declaration *>(p)->refine(scp);
        }
        void compilation_unit::execute(const scope &scp, context &ctx) const
        {
            try
            {
                for (const auto &stmnt : statements)
                    dynamic_cast<const ast::statement *>(stmnt)->execute(scp, ctx);
            }
            catch (const inconsistency_exception &)
            { // we found an inconsistency at root-level..
                throw unsolvable_exception();
            }
        }
    } // namespace ast

    riddle_parser::riddle_parser(std::istream &is) : parser(is) {}
    riddle_parser::~riddle_parser() {}
} // namespace ratio