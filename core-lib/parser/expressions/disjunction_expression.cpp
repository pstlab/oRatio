#include "disjunction_expression.h"
#include "core.h"

namespace ratio
{

namespace ast
{

disjunction_expression::disjunction_expression(const std::vector<expression *> &es) : expressions(es) {}
disjunction_expression::~disjunction_expression()
{
    for (const auto &e : expressions)
        delete e;
}
expr disjunction_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<bool_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(e->evaluate(scp, ctx));
    return scp.get_core().disj(exprs);
}
}
}