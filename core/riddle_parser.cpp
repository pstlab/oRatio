#include "riddle_parser.h"
#include "core.h"
#include "item.h"
#include "type.h"
#include "constructor.h"
#include "method.h"

namespace ratio
{

namespace ast
{

expression::expression() {}
expression::~expression() {}

bool_literal_expression::bool_literal_expression(const bool &l) : riddle::ast::bool_literal_expression(l) {}
bool_literal_expression::~bool_literal_expression() {}
expr bool_literal_expression::evaluate(const scope &scp, context &ctx) const { return scp.get_core().new_bool(literal); }

int_literal_expression::int_literal_expression(const smt::I &l) : riddle::ast::int_literal_expression(l) {}
int_literal_expression::~int_literal_expression() {}
expr int_literal_expression::evaluate(const scope &scp, context &ctx) const { return scp.get_core().new_int(literal); }

real_literal_expression::real_literal_expression(const smt::rational &l) : riddle::ast::real_literal_expression(l) {}
real_literal_expression::~real_literal_expression() {}
expr real_literal_expression::evaluate(const scope &scp, context &ctx) const { return scp.get_core().new_real(literal); }

string_literal_expression::string_literal_expression(const std::string &l) : riddle::ast::string_literal_expression(l) {}
string_literal_expression::~string_literal_expression() {}
expr string_literal_expression::evaluate(const scope &scp, context &ctx) const { return scp.get_core().new_string(literal); }

cast_expression::cast_expression(const std::vector<std::string> &tp, const riddle::ast::expression *const e) : riddle::ast::cast_expression(tp, e) {}
cast_expression::~cast_expression() {}
expr cast_expression::evaluate(const scope &scp, context &ctx) const { return static_cast<const ast::expression *>(xpr)->evaluate(scp, ctx); }

plus_expression::plus_expression(const riddle::ast::expression *const e) : riddle::ast::plus_expression(e) {}
plus_expression::~plus_expression() {}
expr plus_expression::evaluate(const scope &scp, context &ctx) const { return static_cast<const ast::expression *>(xpr)->evaluate(scp, ctx); }

minus_expression::minus_expression(const riddle::ast::expression *const e) : riddle::ast::minus_expression(e) {}
minus_expression::~minus_expression() {}
expr minus_expression::evaluate(const scope &scp, context &ctx) const { return scp.get_core().minus(static_cast<const ast::expression *>(xpr)->evaluate(scp, ctx)); }

not_expression::not_expression(const riddle::ast::expression *const e) : riddle::ast::not_expression(e) {}
not_expression::~not_expression() {}
expr not_expression::evaluate(const scope &scp, context &ctx) const { return scp.get_core().negate(static_cast<const ast::expression *>(xpr)->evaluate(scp, ctx)); }

range_expression::range_expression(const riddle::ast::expression *const min_e, const riddle::ast::expression *const max_e) : riddle::ast::range_expression(min_e, max_e) {}
range_expression::~range_expression() {}
expr range_expression::evaluate(const scope &scp, context &ctx) const
{
    arith_expr min = static_cast<const ast::expression *>(min_xpr)->evaluate(scp, ctx);
    arith_expr max = static_cast<const ast::expression *>(max_xpr)->evaluate(scp, ctx);
    arith_expr var = (min->get_type().get_name().compare(REAL_KEYWORD) == 0 || max->get_type().get_name().compare(REAL_KEYWORD) == 0) ? scp.get_core().new_real() : scp.get_core().new_int();
    scp.get_core().assert_facts({scp.get_core().geq(var, min)->l, scp.get_core().leq(var, max)->l});
    return var;
}

constructor_expression::constructor_expression(const std::vector<std::string> &it, const std::vector<riddle::ast::expression *> &es) : riddle::ast::constructor_expression(it, es) {}
constructor_expression::~constructor_expression() {}
expr constructor_expression::evaluate(const scope &scp, context &ctx) const
{
    scope *s = const_cast<scope *>(&scp);
    for (const auto &tp : instance_type)
        s = &s->get_type(tp);

    std::vector<expr> exprs;
    std::vector<const type *> par_types;
    for (const auto &ex : expressions)
    {
        expr i = static_cast<const ast::expression *>(ex)->evaluate(scp, ctx);
        exprs.push_back(i);
        par_types.push_back(&i->get_type());
    }

    return static_cast<type *>(s)->get_constructor(par_types).new_instance(ctx, exprs);
}

eq_expression::eq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::eq_expression(l, r) {}
eq_expression::~eq_expression() {}
expr eq_expression::evaluate(const scope &scp, context &ctx) const
{
    expr l = static_cast<const ast::expression *>(left)->evaluate(scp, ctx);
    expr r = static_cast<const ast::expression *>(right)->evaluate(scp, ctx);
    return scp.get_core().eq(l, r);
}

neq_expression::neq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::neq_expression(l, r) {}
neq_expression::~neq_expression() {}
expr neq_expression::evaluate(const scope &scp, context &ctx) const
{
    expr l = static_cast<const ast::expression *>(left)->evaluate(scp, ctx);
    expr r = static_cast<const ast::expression *>(right)->evaluate(scp, ctx);
    return scp.get_core().negate(scp.get_core().eq(l, r));
}

lt_expression::lt_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::lt_expression(l, r) {}
lt_expression::~lt_expression() {}
expr lt_expression::evaluate(const scope &scp, context &ctx) const
{
    arith_expr l = static_cast<const ast::expression *>(left)->evaluate(scp, ctx);
    arith_expr r = static_cast<const ast::expression *>(right)->evaluate(scp, ctx);
    return scp.get_core().lt(l, r);
}

leq_expression::leq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::leq_expression(l, r) {}
leq_expression::~leq_expression() {}
expr leq_expression::evaluate(const scope &scp, context &ctx) const
{
    arith_expr l = static_cast<const ast::expression *>(left)->evaluate(scp, ctx);
    arith_expr r = static_cast<const ast::expression *>(right)->evaluate(scp, ctx);
    return scp.get_core().leq(l, r);
}

geq_expression::geq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::geq_expression(l, r) {}
geq_expression::~geq_expression() {}
expr geq_expression::evaluate(const scope &scp, context &ctx) const
{
    arith_expr l = static_cast<const ast::expression *>(left)->evaluate(scp, ctx);
    arith_expr r = static_cast<const ast::expression *>(right)->evaluate(scp, ctx);
    return scp.get_core().geq(l, r);
}

gt_expression::gt_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::gt_expression(l, r) {}
gt_expression::~gt_expression() {}
expr gt_expression::evaluate(const scope &scp, context &ctx) const
{
    arith_expr l = static_cast<const ast::expression *>(left)->evaluate(scp, ctx);
    arith_expr r = static_cast<const ast::expression *>(right)->evaluate(scp, ctx);
    return scp.get_core().gt(l, r);
}

function_expression::function_expression(const std::vector<std::string> &is, const std::string &fn, const std::vector<riddle::ast::expression *> &es) : riddle::ast::function_expression(is, fn, es) {}
function_expression::~function_expression() {}
expr function_expression::evaluate(const scope &scp, context &ctx) const
{
    scope *s = const_cast<scope *>(&scp);
    for (const auto &id : ids)
        s = &s->get_type(id);

    std::vector<expr> exprs;
    std::vector<const type *> par_types;
    for (const auto &ex : expressions)
    {
        expr i = static_cast<const ast::expression *>(ex)->evaluate(scp, ctx);
        exprs.push_back(i);
        par_types.push_back(&i->get_type());
    }

    const method &m = s->get_method(function_name, par_types);
    if (m.get_return_type())
    {
        if (m.get_return_type() == &scp.get_core().get_type(BOOL_KEYWORD))
            return bool_expr(static_cast<bool_item *>(m.invoke(ctx, exprs)));
        else if (m.get_return_type() == &scp.get_core().get_type(INT_KEYWORD) || m.get_return_type() == &scp.get_core().get_type(REAL_KEYWORD))
            return arith_expr(static_cast<arith_item *>(m.invoke(ctx, exprs)));
        else
            return expr(m.invoke(ctx, exprs));
    }
    else
        return scp.get_core().new_bool(true);
}

id_expression::id_expression(const std::vector<std::string> &is) : riddle::ast::id_expression(is) {}
id_expression::~id_expression() {}
expr id_expression::evaluate(const scope &, context &ctx) const
{
    env *c_e = &*ctx;
    for (const auto &id : ids)
        c_e = &*c_e->get(id);
    return expr(static_cast<item *>(c_e));
}

implication_expression::implication_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) : riddle::ast::implication_expression(l, r) {}
implication_expression::~implication_expression() {}
expr implication_expression::evaluate(const scope &scp, context &ctx) const
{
    bool_expr l = static_cast<const ast::expression *>(left)->evaluate(scp, ctx);
    bool_expr r = static_cast<const ast::expression *>(right)->evaluate(scp, ctx);
    return scp.get_core().disj({scp.get_core().negate(l), r});
}

disjunction_expression::disjunction_expression(const std::vector<riddle::ast::expression *> &es) : riddle::ast::disjunction_expression(es) {}
disjunction_expression::~disjunction_expression() {}
expr disjunction_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<bool_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(static_cast<const ast::expression *>(e)->evaluate(scp, ctx));
    return scp.get_core().disj(exprs);
}

conjunction_expression::conjunction_expression(const std::vector<riddle::ast::expression *> &es) : riddle::ast::conjunction_expression(es) {}
conjunction_expression::~conjunction_expression() {}
expr conjunction_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<bool_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(static_cast<const ast::expression *>(e)->evaluate(scp, ctx));
    return scp.get_core().conj(exprs);
}

exct_one_expression::exct_one_expression(const std::vector<riddle::ast::expression *> &es) : riddle::ast::exct_one_expression(es) {}
exct_one_expression::~exct_one_expression() {}
expr exct_one_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<bool_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(static_cast<const ast::expression *>(e)->evaluate(scp, ctx));
    return scp.get_core().exct_one(exprs);
}

addition_expression::addition_expression(const std::vector<riddle::ast::expression *> &es) : riddle::ast::addition_expression(es) {}
addition_expression::~addition_expression() {}
expr addition_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<arith_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(static_cast<const ast::expression *>(e)->evaluate(scp, ctx));
    return scp.get_core().add(exprs);
}

subtraction_expression::subtraction_expression(const std::vector<riddle::ast::expression *> &es) : riddle::ast::subtraction_expression(es) {}
subtraction_expression::~subtraction_expression() {}
expr subtraction_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<arith_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(static_cast<const ast::expression *>(e)->evaluate(scp, ctx));
    return scp.get_core().sub(exprs);
}

multiplication_expression::multiplication_expression(const std::vector<riddle::ast::expression *> &es) : riddle::ast::multiplication_expression(es) {}
multiplication_expression::~multiplication_expression() {}
expr multiplication_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<arith_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(static_cast<const ast::expression *>(e)->evaluate(scp, ctx));
    return scp.get_core().mult(exprs);
}

division_expression::division_expression(const std::vector<riddle::ast::expression *> &es) : riddle::ast::division_expression(es) {}
division_expression::~division_expression() {}
expr division_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<arith_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(static_cast<const ast::expression *>(e)->evaluate(scp, ctx));
    return scp.get_core().div(exprs);
}
} // namespace ast

riddle_parser::riddle_parser(std::istream &is) : parser(is) {}
riddle_parser::~riddle_parser() {}
} // namespace ratio
