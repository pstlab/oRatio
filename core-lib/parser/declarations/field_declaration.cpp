#include "field_declaration.h"
#include "type.h"
#include "field.h"
#include "variable_declaration.h"
#include "context.h"

namespace ratio
{

namespace ast
{

field_declaration::field_declaration(const std::vector<std::string> &tp, const std::vector<variable_declaration *> &ds) : field_type(tp), declarations(ds) {}
field_declaration::~field_declaration()
{
    for (const auto &vd : declarations)
        delete vd;
}

void field_declaration::refine(scope &scp) const
{
    // we add fields to the current scope..
    scope *s = &scp;
    for (const auto &id : field_type)
        s = &s->get_type(id);
    type *tp = static_cast<type *>(s);

    for (const auto &vd : declarations)
        scp.fields.insert({vd->name, new field(*tp, vd->name, vd->xpr)});
}
}
}