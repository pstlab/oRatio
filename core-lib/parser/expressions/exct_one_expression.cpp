#include "exct_one_expression.h"
#include "context.h"
#include "core.h"

namespace ratio
{

namespace ast
{

exct_one_expression::exct_one_expression(const std::vector<expression *> &es) : expressions(es) {}
exct_one_expression::~exct_one_expression()
{
    for (const auto &e : expressions)
        delete e;
}

expr exct_one_expression::evaluate(const scope &scp, context &ctx) const
{
    std::vector<bool_expr> exprs;
    for (const auto &e : expressions)
        exprs.push_back(e->evaluate(scp, ctx));
    return scp.get_core().exct_one(exprs);
}
}
}