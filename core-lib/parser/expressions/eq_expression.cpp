#include "eq_expression.h"
#include "core.h"

namespace ratio
{

namespace ast
{

eq_expression::eq_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
eq_expression::~eq_expression()
{
    delete left;
    delete right;
}

expr eq_expression::evaluate(const scope &scp, context &ctx) const
{
    expr l = left->evaluate(scp, ctx);
    expr r = right->evaluate(scp, ctx);
    return scp.get_core().eq(l, r);
}
}
}