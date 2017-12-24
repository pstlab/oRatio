#include "constructor_declaration.h"
#include "type.h"
#include "field.h"
#include "constructor.h"
#include "expression.h"
#include "statement.h"
#include "context.h"

namespace ratio
{

namespace ast
{

constructor_declaration::constructor_declaration(const std::vector<std::pair<std::vector<std::string>, std::string>> &pars, const std::vector<std::pair<std::string, std::vector<expression *>>> &il, const std::vector<statement *> &stmnts) : parameters(pars), init_list(il), statements(stmnts) {}
constructor_declaration::~constructor_declaration()
{
    for (const auto &i : init_list)
        for (const auto &e : i.second)
            delete e;
    for (const auto &s : statements)
        delete s;
}

void constructor_declaration::refine(scope &scp) const
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

    static_cast<type &>(scp).constructors.push_back(new constructor(scp.get_core(), scp, args, init_list, statements));
}
}
}