#include "gt_expression.h"
#include "core.h"

namespace ratio
{

namespace ast
{

gt_expression::gt_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
gt_expression::~gt_expression()
{
    delete left;
    delete right;
}
expr gt_expression::evaluate(const scope &scp, context &ctx) const
{
    arith_expr l = left->evaluate(scp, ctx);
    arith_expr r = right->evaluate(scp, ctx);
    return scp.get_core().gt(l, r);
}
}
}