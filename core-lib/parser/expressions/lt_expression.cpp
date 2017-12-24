#include "lt_expression.h"
#include "context.h"

namespace ratio
{

namespace ast
{

lt_expression::lt_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
lt_expression::~lt_expression()
{
    delete left;
    delete right;
}
expr lt_expression::evaluate(const scope &scp, context &ctx) const
{
    arith_expr l = left->evaluate(scp, ctx);
    arith_expr r = right->evaluate(scp, ctx);
    return scp.get_core().lt(l, r);
}
}
}