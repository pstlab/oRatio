#include "neq_expression.h"
#include "context.h"

namespace ratio
{

namespace ast
{

neq_expression::neq_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
neq_expression::~neq_expression()
{
    delete left;
    delete right;
}
expr neq_expression::evaluate(const scope &scp, context &ctx) const
{
    expr l = left->evaluate(scp, ctx);
    expr r = right->evaluate(scp, ctx);
    return scp.get_core().negate(scp.get_core().eq(l, r));
}
}
}