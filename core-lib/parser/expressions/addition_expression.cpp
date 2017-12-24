#include "addition_expression.h"
#include "core.h"

namespace ratio
{

namespace ast
{

addition_expression::addition_expression(const std::vector<expression *> &es) : expressions(es) {}
addition_expression::~addition_expression()
{
    for (const auto &e : expressions)
        delete e;
}

expr addition_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<arith_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(e->evaluate(scp, ctx));
    return scp.get_core().add(exprs);
}
}
}