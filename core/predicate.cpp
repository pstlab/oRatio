#include "predicate.h"
#include "field.h"
#include "context.h"
#include <queue>

namespace ratio
{

predicate::predicate(core &cr, scope &scp, const std::string &name, const std::vector<field *> &args) : type(cr, scp, name), args(args)
{
    if (type *t = dynamic_cast<type *>(&scp))
        new_fields({new field(*t, THIS_KEYWORD, true)});
    new_fields(args);
}

predicate::~predicate() {}
} // namespace ratio