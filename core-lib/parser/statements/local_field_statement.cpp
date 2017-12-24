#include "local_field_statement.h"
#include "expression.h"
#include "type.h"
#include "core.h"
#include "field.h"
#include "item.h"

namespace ratio
{

namespace ast
{

local_field_statement::local_field_statement(const std::vector<std::string> &ft, const std::string &n, const expression *const e) : field_type(ft), name(n), xpr(e) {}
local_field_statement::~local_field_statement() { delete xpr; }

void local_field_statement::execute(const scope &scp, context &ctx) const
{
    if (xpr)
        ctx->items.insert({name, xpr->evaluate(scp, ctx)});
    else
    {
        scope *s = const_cast<scope *>(&scp);
        for (const auto &tp : field_type)
            s = &s->get_type(tp);
        type *t = static_cast<type *>(s);
        if (t->primitive)
            ctx->items.insert({name, t->new_instance(ctx)});
        else
            ctx->items.insert({name, t->new_existential()});
    }
    if (const core *c = dynamic_cast<const core *>(&scp)) // we create fields for root items..
        const_cast<core *>(c)->fields.insert({name, new field(ctx->items.at(name)->tp, name)});
}
}
}