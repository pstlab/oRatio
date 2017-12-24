#include "constructor.h"
#include "field.h"
#include "type.h"
#include "item.h"
#include "statement.h"
#include "expression.h"
#include <cassert>

namespace ratio
{

constructor::constructor(core &cr, scope &scp, const std::vector<field *> &args, const std::vector<std::pair<std::string, std::vector<ast::expression *>>> &il, const std::vector<ast::statement *> &stmnts) : scope(cr, scp), args(args), init_list(il), statements(stmnts)
{
    add_fields({new field(static_cast<type &>(scp), THIS_KEYWORD, nullptr, true)});
    add_fields(args);
}

constructor::~constructor() {}

expr constructor::new_instance(context &ctx, const std::vector<expr> &exprs)
{
    assert(args.size() == exprs.size());

    type &t = static_cast<type &>(scp);
    expr i = t.new_instance(ctx);

    invoke(*i, exprs);

    return i;
}

void constructor::invoke(item &itm, const std::vector<expr> &exprs)
{
    context ctx(new env(cr, &itm));
    ctx->items.insert({THIS_KEYWORD, expr(&itm)});
    for (size_t i = 0; i < args.size(); i++)
        ctx->items.insert({args.at(i)->name, exprs.at(i)});

    // we initialize the supertypes..
    size_t il_idx = 0;
    for (const auto &st : static_cast<type &>(scp).get_supertypes())
        if (il_idx < init_list.size() && init_list.at(il_idx).first.compare(st->name) == 0) // explicit supertype constructor invocation..
        {
            std::vector<expr> c_exprs;
            std::vector<const type *> par_types;
            for (const auto &ex : init_list.at(il_idx).second)
            {
                expr c_expr = ex->evaluate(*this, ctx);
                c_exprs.push_back(c_expr);
                par_types.push_back(&c_expr->tp);
            }

            // we assume that the constructor exists..
            st->get_constructor(par_types).invoke(itm, c_exprs);
            il_idx++;
        }
        else // implicit supertype (default) constructor invocation..
        {
            // we assume that the default constructor exists..
            st->get_constructor({}).invoke(itm, {});
        }

    // we procede with the assignment list..
    for (; il_idx < init_list.size(); il_idx++)
    {
        assert(init_list[il_idx].second.size() == 1);
        itm.items.insert({init_list[il_idx].first, init_list[il_idx].second[0]->evaluate(*this, ctx)});
    }

    // we instantiate the uninstantiated fields..
    for (const auto &f : scp.get_fields())
        if (!f.second->synthetic && itm.items.find((f.second->name)) == itm.items.end())
        {
            // the field is uninstantiated..
            if (f.second->xpr)
                itm.items.insert({f.second->name, f.second->xpr->evaluate(*this, ctx)});
            else
            {
                type &tp = const_cast<type &>(f.second->tp);
                if (tp.primitive)
                    itm.items.insert({f.second->name, tp.new_instance(ctx)});
                else
                    itm.items.insert({f.second->name, tp.new_existential()});
            }
        }

    // finally, we execute the constructor body..
    for (const auto &s : statements)
        s->execute(*this, ctx);
}
}