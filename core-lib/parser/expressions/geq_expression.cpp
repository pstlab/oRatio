#include "geq_expression.h"
#include "core.h"

namespace ratio
{

namespace ast
{

geq_expression::geq_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
geq_expression::~geq_expression()
{
    delete left;
    delete right;
}
expr geq_expression::evaluate(const scope &scp, context &ctx) const
{
    arith_expr l = left->evaluate(scp, ctx);
    arith_expr r = right->evaluate(scp, ctx);
    return scp.get_core().geq(l, r);
}
}
}