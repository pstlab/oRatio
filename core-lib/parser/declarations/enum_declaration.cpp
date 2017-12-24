#include "enum_declaration.h"

namespace ratio
{

namespace ast
{

enum_declaration::enum_declaration(const std::string &n, const std::vector<std::string> &es, const std::vector<std::vector<std::string>> &trs) : type_declaration(n), enums(es), type_refs(trs) {}
enum_declaration::~enum_declaration() {}

void enum_declaration::declare(scope &scp) const
{
    // A new enum type has been declared..
    enum_type *et = new enum_type(scp.get_core(), scp, name);

    // We add the enum values..
    for (const auto &e : enums)
        et->instances.push_back(scp.get_core().new_string(e));

    if (core *c = dynamic_cast<core *>(&scp))
        c->types.insert({name, et});
    else if (type *t = dynamic_cast<type *>(&scp))
        t->types.insert({name, et});
}

void enum_declaration::refine(scope &scp) const
{
    if (!type_refs.empty())
    {
        enum_type *et = static_cast<enum_type *>(&scp.get_type(name));
        for (const auto &tr : type_refs)
        {
            scope *s = &scp;
            for (const auto &id : tr)
                s = &s->get_type(id);
            et->enums.push_back(static_cast<enum_type *>(s));
        }
    }
}
}
}