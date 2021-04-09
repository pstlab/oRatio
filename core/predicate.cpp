#include "predicate.h"
#include "field.h"
#include "atom.h"
#include "core_parser.h"
#include <queue>

namespace ratio
{
    CORE_EXPORT predicate::predicate(core &cr, scope &scp, const std::string &name, const std::vector<const field *> &args, const std::vector<const riddle::ast::statement *> &stmnts) : type(cr, scp, name), args(args), statements(stmnts)
    {
        if (type *t = dynamic_cast<type *>(&scp))
            new_fields({new field(*t, TAU, nullptr)});
        new_fields(args);
    }
    CORE_EXPORT predicate::~predicate() {}

    CORE_EXPORT expr predicate::new_instance(context &ctx) noexcept
    {
        atom *a = new atom(get_core(), context(ctx), *this);
        // we add the new atom to the instances of this predicate and to the instances of all the super-predicates..
        std::queue<predicate *> q;
        q.push(this);
        while (!q.empty())
        {
            q.front()->instances.push_back(a);
            for (const auto &st : q.front()->supertypes)
                q.push(static_cast<predicate *>(st));
            q.pop();
        }

        return expr(a);
    }

    CORE_EXPORT void predicate::apply_rule(atom &a) const
    {
        for (const auto &sp : supertypes)
            static_cast<predicate *>(sp)->apply_rule(a);

        context ctx(new env(get_core(), context(&a)));
        ctx->exprs.emplace(THIS_KEYWORD, &a);
        for (const auto &s : statements)
            dynamic_cast<const ast::statement *>(s)->execute(*this, ctx);
    }
} // namespace ratio