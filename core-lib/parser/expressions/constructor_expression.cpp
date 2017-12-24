#include "constructor_expression.h"
#include "item.h"
#include "type.h"
#include "constructor.h"

namespace ratio
{

namespace ast
{

constructor_expression::constructor_expression(const std::vector<std::string> &it, const std::vector<expression *> &es) : instance_type(it), expressions(es) {}
constructor_expression::~constructor_expression()
{
    for (const auto &e : expressions)
        delete e;
}

expr constructor_expression::evaluate(const scope &scp, context &ctx) const
{
    scope *s = const_cast<scope *>(&scp);
    for (const auto &tp : instance_type)
        s = &s->get_type(tp);

    std::vector<expr> exprs;
    std::vector<const type *> par_types;
    for (const auto &ex : expressions)
    {
        expr i = ex->evaluate(scp, ctx);
        exprs.push_back(i);
        par_types.push_back(&i->tp);
    }

    return static_cast<type *>(s)->get_constructor(par_types).new_instance(ctx, exprs);
}
}
}