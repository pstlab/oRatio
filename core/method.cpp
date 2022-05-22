#include "method.h"
#include "type.h"
#include "field.h"
#include "item.h"
#include "core_parser.h"
#include <cassert>

namespace ratio
{
    method::method(scope &scp, const std::optional<type *> &return_type, const std::string &name, const std::vector<const field *> &args, const std::vector<const riddle::ast::statement *> &stmnts) : scope(scp), return_type(return_type), name(name), args(args), statements(stmnts)
    {
        if (type *t = dynamic_cast<type *>(&scp))
            new_fields({new field(*t, THIS_KEYWORD, nullptr, true)});
        if (return_type.has_value())
            new_fields({new field(*return_type.value(), RETURN_KEYWORD, nullptr, true)});
        new_fields(args);
    }

    std::optional<item *> method::invoke(context &ctx, const std::vector<expr> &exprs)
    {
        assert(args.size() == exprs.size());
        context c_ctx(new env(get_core(), context(ctx)));
        for (size_t i = 0; i < args.size(); ++i)
            c_ctx->exprs.emplace(args.at(i)->get_name(), exprs.at(i));

        for (const auto &s : statements)
            dynamic_cast<const ast::statement *>(s)->execute(*this, c_ctx);

        if (return_type.has_value())
            return &*c_ctx->exprs.at(RETURN_KEYWORD);
        else
            return nullptr;
    }
} // namespace ratio