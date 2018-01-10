#include "conjunction.h"
#include "env.h"
#include "statement.h"

namespace ratio
{

conjunction::conjunction(core &cr, scope &scp, const smt::rational &cst, const std::vector<const ast::statement *> &stmnts) : scope(cr, scp), cost(cst), statements(stmnts) {}
conjunction::~conjunction() {}

void conjunction::apply(context &ctx) const
{
    context c_ctx(new env(cr, ctx));
    for (const auto &s : statements)
        s->execute(*this, c_ctx);
}
}