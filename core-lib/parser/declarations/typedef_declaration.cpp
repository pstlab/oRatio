#include "typedef_declaration.h"
#include "typedef_type.h"
#include "core.h"
#include "expression.h"

namespace ratio
{

namespace ast
{
typedef_declaration::typedef_declaration(const std::string &n, const std::string &pt, const expression *const e) : type_declaration(n), primitive_type(pt), xpr(e) {}
typedef_declaration::~typedef_declaration() { delete xpr; }

void typedef_declaration::declare(scope &scp) const
{
    // A new typedef type has been declared..
    typedef_type *td = new typedef_type(scp.get_core(), scp, name, scp.get_type(primitive_type), xpr);

    if (core *c = dynamic_cast<core *>(&scp))
        c->types.insert({name, td});
    else if (type *t = dynamic_cast<type *>(&scp))
        t->types.insert({name, td});
}
}
}