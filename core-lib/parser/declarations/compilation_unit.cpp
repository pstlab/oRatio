#include "compilation_unit.h"
#include "method.h"
#include "predicate.h"
#include "statement.h"

namespace ratio
{

namespace ast
{

compilation_unit::compilation_unit(const std::vector<method_declaration *> &ms, const std::vector<predicate_declaration *> &ps, const std::vector<type_declaration *> &ts, const std::vector<statement *> &stmnts) : methods(ms), predicates(ps), types(ts), statements(stmnts) {}
compilation_unit::~compilation_unit()
{
    for (const auto &t : types)
        delete t;
    for (const auto &m : methods)
        delete m;
    for (const auto &p : predicates)
        delete p;
    for (const auto &s : statements)
        delete s;
}

void compilation_unit::declare(scope &scp) const
{
    for (const auto &t : types)
        t->declare(scp);
}

void compilation_unit::refine(scope &scp) const
{
    for (const auto &t : types)
        t->refine(scp);
    for (const auto &m : methods)
        m->refine(scp);
    for (const auto &p : predicates)
        p->refine(scp);
}

void compilation_unit::execute(const scope &scp, context &ctx) const
{
    for (const auto &stmnt : statements)
        stmnt->execute(scp, ctx);
}
}
}