#include "expression_statement.h"
#include "expression.h"
#include "core.h"
#include "item.h"

namespace ratio
{

namespace ast
{

expression_statement::expression_statement(const expression *const e) : xpr(e) {}
expression_statement::~expression_statement() { delete xpr; }

void expression_statement::execute(const scope &scp, context &ctx) const
{
    bool_expr be = xpr->evaluate(scp, ctx);
    if (scp.get_core().sat_cr.value(be->l) != smt::False)
        scp.get_core().assert_facts({be->l});
    else
        throw inconsistency_exception();
}
}
}