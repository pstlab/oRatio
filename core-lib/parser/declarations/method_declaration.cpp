#include "method_declaration.h"

namespace ratio
{

namespace ast
{

method_declaration::method_declaration(const std::vector<std::string> &rt, const std::string &n, const std::vector<std::pair<std::vector<std::string>, std::string>> &pars, const std::vector<statement *> &stmnts) : return_type(rt), name(n), parameters(pars), statements(stmnts) {}
method_declaration::~method_declaration()
{
    for (const auto &s : statements)
        delete s;
}

void method_declaration::refine(scope &scp) const
{
    type *rt = nullptr;
    if (!return_type.empty())
    {
        scope *s = &scp;
        for (const auto &id : return_type)
            s = &s->get_type(id);
        rt = static_cast<type *>(s);
    }

    std::vector<field *> args;
    for (const auto &par : parameters)
    {
        scope *s = &scp;
        for (const auto &id : par.first)
            s = &s->get_type(id);
        type *tp = static_cast<type *>(s);
        args.push_back(new field(*tp, par.second));
    }

    method *m = new method(scp.get_core(), scp, rt, name, args, statements);

    if (core *c = dynamic_cast<core *>(&scp))
    {
        if (c->methods.find(name) == c->methods.end())
            c->methods.insert({name, *new std::vector<method *>()});
        c->methods.at(name).push_back(m);
    }
    else if (type *t = dynamic_cast<type *>(&scp))
    {
        if (t->methods.find(name) == t->methods.end())
            t->methods.insert({name, *new std::vector<method *>()});
        t->methods.at(name).push_back(m);
    }
}
}
}