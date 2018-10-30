#include "method.h"
#include "type.h"
#include "field.h"
#include "item.h"
#include <cassert>

namespace ratio
{

method::method(core &cr, scope &scp, const type *const return_type, const std::string &name, const std::vector<field *> &args) : scope(cr, scp), return_type(return_type), name(name), args(args)
{
    if (type *t = dynamic_cast<type *>(&scp))
        new_fields({new field(*t, THIS_KEYWORD, true)});
    if (return_type)
        new_fields({new field(*return_type, RETURN_KEYWORD, true)});
    new_fields(args);
}

method::~method() {}

item *method::invoke(context &ctx, const std::vector<expr> &exprs) const
{
    assert(args.size() == exprs.size());
    context c_ctx(new env(get_core(), ctx));
    for (size_t i = 0; i < args.size(); i++)
        c_ctx->exprs.insert({args.at(i)->get_name(), exprs.at(i)});

    // TODO: execute the statements..

    if (return_type)
        return &*c_ctx->exprs.at(RETURN_KEYWORD);
    else
        return nullptr;
}
} // namespace ratio
