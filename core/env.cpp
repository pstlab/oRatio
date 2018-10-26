#include "env.h"

namespace ratio
{

env::env(core &cr, const context ctx) : cr(cr), ctx(ctx) {}

env::~env() {}

expr env::get(const std::string &name) const
{
    const auto at_xpr = exprs.find(name);
    if (at_xpr != exprs.end())
        return at_xpr->second;

    // if not here, check any enclosing environment..
    return ctx->get(name);
}
} // namespace ratio
