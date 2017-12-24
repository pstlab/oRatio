#include "plus_expression.h"
#include "core.h"

namespace ratio
{

namespace ast
{

plus_expression::plus_expression(const expression *const e) : xpr(e) {}
plus_expression::~plus_expression() { delete xpr; }

expr plus_expression::evaluate(const scope &scp, context &ctx) const { return xpr->evaluate(scp, ctx); }
}
}