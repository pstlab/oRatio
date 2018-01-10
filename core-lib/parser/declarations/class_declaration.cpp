#include "class_declaration.h"
#include "type.h"
#include "core.h"
#include "constructor.h"
#include "field_declaration.h"
#include "constructor_declaration.h"
#include "method_declaration.h"
#include "predicate_declaration.h"
#include "type_declaration.h"

namespace ratio
{

namespace ast
{

class_declaration::class_declaration(const std::string &n, const std::vector<std::vector<std::string>> &bcs, const std::vector<field_declaration *> &fs, const std::vector<constructor_declaration *> &cs, const std::vector<method_declaration *> &ms, const std::vector<predicate_declaration *> &ps, const std::vector<type_declaration *> &ts) : type_declaration(n), base_classes(bcs), fields(fs), constructors(cs), methods(ms), predicates(ps), types(ts) {}
class_declaration::~class_declaration()
{
    for (const auto &f : fields)
        delete f;
    for (const auto &c : constructors)
        delete c;
    for (const auto &m : methods)
        delete m;
    for (const auto &p : predicates)
        delete p;
    for (const auto &t : types)
        delete t;
}

void class_declaration::declare(scope &scp) const
{
    // A new type has been declared..
    type *tp = new type(scp.get_core(), scp, name);

    if (core *c = dynamic_cast<core *>(&scp))
        c->types.insert({name, tp});
    else if (type *t = dynamic_cast<type *>(&scp))
        t->types.insert({name, tp});

    for (const auto &c_tp : types)
        c_tp->declare(*tp);
}

void class_declaration::refine(scope &scp) const
{
    type &tp = scp.get_type(name);
    for (const auto &bc : base_classes)
    {
        scope *s = &scp;
        for (const auto &id : bc)
            s = &s->get_type(id);
        tp.supertypes.push_back(static_cast<type *>(s));
    }

    for (const auto &f : fields)
        f->refine(tp);

    if (constructors.empty())
        tp.constructors.push_back(new constructor(scp.get_core(), tp, {}, {}, {})); // we add a default constructor..
    else
        for (const auto &c : constructors)
            c->refine(tp);

    for (const auto &m : methods)
        m->refine(tp);
    for (const auto &p : predicates)
        p->refine(tp);
    for (const auto &t : types)
        t->refine(tp);
}
}
}