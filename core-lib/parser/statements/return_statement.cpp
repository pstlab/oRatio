#include "return_statement.h"

namespace ratio
{

namespace ast
{

return_statement::return_statement(const expression *const e) : xpr(e) {}
return_statement::~return_statement() { delete xpr; }

void return_statement::execute(const scope &scp, context &ctx) const { ctx->items.insert({RETURN_KEYWORD, xpr->evaluate(scp, ctx)}); }
}
}