#include "disjunction_statement.h"
#include "statement.h"
#include "expression.h"
#include "conjunction.h"
#include "disjunction.h"
#include "item.h"
#include "core.h"

namespace ratio
{

namespace ast
{

disjunction_statement::disjunction_statement(const std::vector<std::pair<std::vector<const statement *>, const expression *const>> &conjs) : conjunctions(conjs) {}
disjunction_statement::~disjunction_statement()
{
    for (const auto &c : conjunctions)
    {
        for (const auto &s : c.first)
            delete s;
        delete c.second;
    }
}

void disjunction_statement::execute(const scope &scp, context &ctx) const
{
    std::vector<const conjunction *> cs;
    for (const auto &c : conjunctions)
    {
        smt::rational cost(1);
        if (c.second)
        {
            arith_expr a_xpr = c.second->evaluate(scp, ctx);
            cost = a_xpr->l.known_term;
        }
        cs.push_back(new conjunction(scp.get_core(), const_cast<scope &>(scp), cost, c.first));
    }
    disjunction *d = new disjunction(scp.get_core(), const_cast<scope &>(scp), cs);
    scp.get_core().new_disjunction(ctx, *d);
}
}
}