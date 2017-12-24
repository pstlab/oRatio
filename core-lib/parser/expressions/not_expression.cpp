#include "not_expression.h"
#include "core.h"

namespace ratio
{

namespace ast
{
not_expression::not_expression(const expression *const e) : xpr(e) {}
not_expression::~not_expression() { delete xpr; }

expr not_expression::evaluate(const scope &scp, context &ctx) const { return scp.get_core().negate(xpr->evaluate(scp, ctx)); }
}
}