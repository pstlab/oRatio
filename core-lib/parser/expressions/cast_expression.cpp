#include "cast_expression.h"
#include "context.h"

namespace ratio
{

namespace ast
{

cast_expression::cast_expression(const std::vector<std::string> &tp, const expression *const e) : cast_to_type(tp), xpr(e) {}
cast_expression::~cast_expression() { delete xpr; }

expr cast_expression::evaluate(const scope &scp, context &ctx) const { return xpr->evaluate(scp, ctx); }
}
}