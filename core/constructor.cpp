#include "constructor.h"
#include "field.h"
#include "type.h"
#include "item.h"
#include "core_parser.h"
#include <cassert>

namespace ratio
{
    CORE_EXPORT constructor::constructor(core &cr, scope &scp, const std::vector<const field *> &args, const std::vector<std::pair<const std::string, const std::vector<const riddle::ast::expression *>>> &il, const std::vector<const riddle::ast::statement *> &stmnts) : scope(cr, scp), args(args), init_list(il), statements(stmnts)
    {
        new_fields({new field(static_cast<type &>(scp), THIS_KEYWORD, nullptr, true)});
        new_fields(args);
    }
    CORE_EXPORT constructor::~constructor() {}

    expr constructor::new_instance(context &ctx, const std::vector<expr> &exprs) const noexcept
    {
        assert(args.size() == exprs.size());

        type &t = static_cast<type &>(get_scope());
        expr i = t.new_instance(ctx);

        invoke(*i, exprs);

        return i;
    }

    void constructor::invoke(item &itm, const std::vector<expr> &exprs) const
    {
        context ctx(new env(get_core(), context(&itm)));
        ctx->exprs.emplace(THIS_KEYWORD, expr(&itm));
        for (size_t i = 0; i < args.size(); ++i)
            ctx->exprs.emplace(args.at(i)->get_name(), exprs.at(i));

        // we initialize the supertypes..
        size_t il_idx = 0;
        for (const auto &st : static_cast<type &>(get_scope()).get_supertypes())
            if (il_idx < init_list.size() && init_list.at(il_idx).first.compare(st->get_name()) == 0) // explicit supertype constructor invocation..
            {
                std::vector<expr> c_exprs;
                std::vector<const type *> par_types;
                for (const auto &ex : init_list.at(il_idx).second)
                {
                    expr c_expr = dynamic_cast<const ast::expression *>(ex)->evaluate(*this, ctx);
                    c_exprs.push_back(c_expr);
                    par_types.push_back(&c_expr->get_type());
                }

                // we assume that the constructor exists..
                st->get_constructor(par_types).invoke(itm, c_exprs);
                il_idx++;
            }
            else                                         // implicit supertype (default) constructor invocation..
                st->get_constructor({}).invoke(itm, {}); // we assume that the default constructor exists..

        // we procede with the assignment list..
        for (; il_idx < init_list.size(); il_idx++)
        {
            assert(init_list[il_idx].second.size() == 1);
            itm.exprs.emplace(init_list[il_idx].first, dynamic_cast<const ast::expression *>(init_list[il_idx].second[0])->evaluate(*this, ctx));
        }

        // we instantiate the uninstantiated fields..
        for (const auto &[f_name, f] : get_scope().get_fields())
            if (!f->is_synthetic() && !itm.exprs.count((f->get_name())))
            { // the field is uninstantiated..
                if (f->get_expression())
                    itm.exprs.emplace(f->get_name(), dynamic_cast<const ast::expression *>(f->get_expression())->evaluate(*this, ctx));
                else
                {
                    type &tp = const_cast<type &>(f->get_type());
                    if (tp.is_primitive())
                        itm.exprs.emplace(f->get_name(), tp.new_instance(ctx));
                    else
                        itm.exprs.emplace(f->get_name(), tp.new_existential());
                }
            }

        // finally, we execute the constructor body..
        for (const auto &s : statements)
            dynamic_cast<const ast::statement *>(s)->execute(*this, ctx);
    }
} // namespace ratio