#include "minus_expression.h"
#include "context.h"

namespace ratio
{

namespace ast
{

minus_expression::minus_expression(const expression *const e) : xpr(e) {}
minus_expression::~minus_expression() { delete xpr; }

expr minus_expression::evaluate(const scope &scp, context &ctx) const { return scp.get_core().minus(xpr->evaluate(scp, ctx)); }
}
}