#include "conjunction_expression.h"
#include "context.h"
#include "scope.h"

namespace ratio
{

namespace ast
{

conjunction_expression::conjunction_expression(const std::vector<expression *> &es) : expressions(es) {}
conjunction_expression::~conjunction_expression()
{
    for (const auto &e : expressions)
        delete e;
}
expr conjunction_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<bool_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(e->evaluate(scp, ctx));
    return scp.get_core().conj(exprs);
}
}
}