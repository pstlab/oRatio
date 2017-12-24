#include "leq_expression.h"
#include "context.h"

namespace ratio
{

namespace ast
{

leq_expression::leq_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
leq_expression::~leq_expression()
{
    delete left;
    delete right;
}
expr leq_expression::evaluate(const scope &scp, context &ctx) const
{
    arith_expr l = left->evaluate(scp, ctx);
    arith_expr r = right->evaluate(scp, ctx);
    return scp.get_core().leq(l, r);
}
}
}