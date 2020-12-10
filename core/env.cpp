#include "env.h"
#include "core.h"
#include "item.h"
#include "type.h"
#include <cassert>

using namespace smt;

namespace ratio
{

    env::env(core &cr, const context ctx) : ref_count(this == &cr ? 2 : 0), cr(cr), ctx(ctx) {}

    env::~env() { assert(!ref_count || (this == &*ctx && ref_count)); }

    expr env::get(const std::string &name) const
    {
        if (const auto at_xpr = exprs.find(name); at_xpr != exprs.end())
            return at_xpr->second;

        // if not here, check any enclosing environment..
        return ctx->get(name);
    }

    json env::to_json() const noexcept
    {
        std::vector<json> j_exprs;
        for (const auto &c_expr : exprs)
        {
            json j_env;
            j_env->set("name", new string_val(c_expr.first));
            j_env->set("type", new string_val(c_expr.second->get_type().get_full_name()));
            j_env->set("value", c_expr.second->value_to_json());
            j_exprs.push_back(j_env);
        }
        return new array_val(j_exprs);
    }
} // namespace ratio