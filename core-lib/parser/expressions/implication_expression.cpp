#include "implication_expression.h"
#include "core.h"

namespace ratio
{

namespace ast
{

implication_expression::implication_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
implication_expression::~implication_expression()
{
    delete left;
    delete right;
}
expr implication_expression::evaluate(const scope &scp, context &ctx) const
{
    bool_expr l = left->evaluate(scp, ctx);
    bool_expr r = right->evaluate(scp, ctx);
    return scp.get_core().disj({scp.get_core().negate(l), r});
}
}
}