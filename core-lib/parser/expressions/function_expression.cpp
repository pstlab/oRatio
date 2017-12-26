#include "function_expression.h"
#include "type.h"
#include "method.h"
#include "core.h"
#include "item.h"

namespace ratio
{

namespace ast
{

function_expression::function_expression(const std::vector<std::string> &is, const std::string &fn, const std::vector<expression *> &es) : ids(is), function_name(fn), expressions(es) {}
function_expression::~function_expression()
{
    for (const auto &e : expressions)
        delete e;
}

expr function_expression::evaluate(const scope &scp, context &ctx) const
{
    scope *s = const_cast<scope *>(&scp);
    for (const auto &id : ids)
        s = &s->get_type(id);

    std::vector<expr> exprs;
    std::vector<const type *> par_types;
    for (const auto &ex : expressions)
    {
        expr i = ex->evaluate(scp, ctx);
        exprs.push_back(i);
        par_types.push_back(&i->tp);
    }

    const method &m = s->get_method(function_name, par_types);
    if (m.return_type)
    {
        if (m.return_type == &scp.get_core().get_type(BOOL_KEYWORD))
            return bool_expr(static_cast<bool_item *>(m.invoke(ctx, exprs)));
        else if (m.return_type == &scp.get_core().get_type(INT_KEYWORD) || m.return_type == &scp.get_core().get_type(REAL_KEYWORD))
            return arith_expr(static_cast<arith_item *>(m.invoke(ctx, exprs)));
        else
            return expr(m.invoke(ctx, exprs));
    }
    else
        return scp.get_core().new_bool(true);
}
}
}