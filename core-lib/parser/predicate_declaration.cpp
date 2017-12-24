#include "variable_declaration.h"

namespace ratio
{

namespace ast
{

predicate_declaration::predicate_declaration(const std::string &n, const std::vector<std::pair<std::vector<std::string>, std::string>> &pars, const std::vector<std::vector<std::string>> &pl, const std::vector<statement *> &stmnts) : name(n), parameters(pars), predicate_list(pl), statements(stmnts) {}
predicate_declaration::~predicate_declaration()
{
    for (const auto &s : statements)
        delete s;
}
void predicate_declaration::refine(scope &scp) const
{
    std::vector<field *> args;
    for (const auto &par : parameters)
    {
        scope *s = &scp;
        for (const auto &id : par.first)
            s = &s->get_type(id);
        type *tp = static_cast<type *>(s);
        args.push_back(new field(*tp, par.second));
    }

    predicate *p = new predicate(scp.get_core(), scp, name, args, statements);

    // we add the supertypes.. notice that we do not support forward declaration for predicate supertypes!!
    for (const auto &sp : predicate_list)
    {
        scope *s = &scp;
        for (const auto &id : sp)
            s = &s->get_predicate(id);
        p->supertypes.push_back(static_cast<predicate *>(s));
    }

    if (core *c = dynamic_cast<core *>(&scp))
        c->predicates.insert({name, p});
    else if (type *t = dynamic_cast<type *>(&scp))
    {
        t->predicates.insert({name, p});
        std::queue<type *> q;
        q.push(t);
        while (!q.empty())
        {
            q.front()->new_predicate(*p);
            for (const auto &st : q.front()->supertypes)
                q.push(st);
            q.pop();
        }
    }
}
}
}