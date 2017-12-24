#include "assignment_statement.h"
#include "expression.h"
#include "item.h"

namespace ratio
{

namespace ast
{

assignment_statement::assignment_statement(const std::vector<std::string> &is, const std::string &i, const expression *const e) : ids(is), id(i), xpr(e) {}
assignment_statement::~assignment_statement() { delete xpr; }

void assignment_statement::execute(const scope &scp, context &ctx) const
{
    env *c_e = &*ctx;
    for (const auto &c_id : ids)
        c_e = &*c_e->get(c_id);
    c_e->items.insert({id, xpr->evaluate(scp, ctx)});
}
}
}