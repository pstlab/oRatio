#include "subtraction_expression.h"
#include "core.h"

namespace ratio
{

namespace ast
{

subtraction_expression::subtraction_expression(const std::vector<expression *> &es) : expressions(es) {}
subtraction_expression::~subtraction_expression()
{
    for (const auto &e : expressions)
        delete e;
}

expr subtraction_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<arith_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(e->evaluate(scp, ctx));
    return scp.get_core().sub(exprs);
}
}
}