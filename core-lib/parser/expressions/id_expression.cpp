#include "id_expression.h"
#include "context.h"

namespace ratio
{

namespace ast
{

id_expression::id_expression(const std::vector<std::string> &is) : ids(is) {}
id_expression::~id_expression() {}

expr id_expression::evaluate(const scope &, context &ctx) const
{
    env *c_e = &*ctx;
    for (const auto &id : ids)
        c_e = &*c_e->get(id);
    return expr(static_cast<item *>(c_e));
}
}
}