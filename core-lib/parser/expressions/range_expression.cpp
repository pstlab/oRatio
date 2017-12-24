#include "range_expression.h"
#include "context.h"

namespace ratio
{

namespace ast
{

range_expression::range_expression(const expression *const min_e, const expression *const max_e) : min_xpr(min_e), max_xpr(max_e) {}
range_expression::~range_expression()
{
    delete min_xpr;
    delete max_xpr;
}

expr range_expression::evaluate(const scope &scp, context &ctx) const
{
    arith_expr min = min_xpr->evaluate(scp, ctx);
    arith_expr max = max_xpr->evaluate(scp, ctx);
    arith_expr var = (min->tp.name.compare(REAL_KEYWORD) == 0 || max->tp.name.compare(REAL_KEYWORD) == 0) ? scp.get_core().new_real() : scp.get_core().new_int();
    scp.get_core().assert_facts({scp.get_core().geq(var, min)->l, scp.get_core().leq(var, max)->l});
    return var;
}
}
}