#include "constructor.h"
#include "field.h"
#include "type.h"
#include "item.h"
#include <cassert>

namespace ratio
{

constructor::constructor(core &cr, scope &scp, const std::vector<field *> &args) : scope(cr, scp), args(args)
{
    new_fields({new field(static_cast<type &>(scp), THIS_KEYWORD, true)});
    new_fields(args);
}

constructor::~constructor() {}

expr constructor::new_instance(context &ctx, const std::vector<expr> &exprs) const
{
    assert(args.size() == exprs.size());

    type &t = static_cast<type &>(get_scope());
    expr i = t.new_instance(ctx);

    invoke(*i, exprs);

    return i;
}

void constructor::invoke(item &itm, const std::vector<expr> &exprs) const {}
} // namespace ratio
