#include "multiplication_expression.h"
#include "context.h"

namespace ratio
{

namespace ast
{

multiplication_expression::multiplication_expression(const std::vector<expression *> &es) : expressions(es) {}
multiplication_expression::~multiplication_expression()
{
    for (const auto &e : expressions)
        delete e;
}
expr multiplication_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<arith_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(e->evaluate(scp, ctx));
    return scp.get_core().mult(exprs);
}
}
}