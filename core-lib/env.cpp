#include "env.h"
#include "core.h"
#include <cassert>

namespace ratio
{

env::env(core &cr, const context ctx) : ref_count(this == &cr ? 2 : 0), cr(cr), ctx(ctx) {}
env::~env() { assert(!ref_count || (this == &*ctx && ref_count)); }

expr env::get(const std::string &name) const
{
    const auto at_itm = items.find(name);
    if (at_itm != items.end())
        return at_itm->second;

    // if not here, check any enclosing environment
    return ctx->get(name);
}
}