#include "division_expression.h"
#include "context.h"

namespace ratio
{

namespace ast
{

division_expression::division_expression(const std::vector<expression *> &es) : expressions(es) {}
division_expression::~division_expression()
{
    for (const auto &e : expressions)
        delete e;
}

expr division_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<arith_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(e->evaluate(scp, ctx));
    return scp.get_core().div(exprs);
}
}
}