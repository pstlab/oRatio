#include "type.h"
#include "item.h"
#include "core.h"

namespace ratio
{

type::type(core &cr, scope &scp, const std::string &name, bool primitive) : scope(cr, scp), name(name), primitive(primitive) {}

type::~type() {}

field &type::get_field(const std::string &f_name) const
{
    const auto at_f = fields.find(f_name);
    if (at_f != fields.end())
        return *at_f->second;

    // if not here, check any enclosing scope
    try
    {
        return scp.get_field(f_name);
    }
    catch (const std::out_of_range &)
    {
        // if not in any enclosing scope, check any superclass
        for (const auto &st : supertypes)
            try
            {
                return st->get_field(f_name);
            }
            catch (const std::out_of_range &)
            {
            }
    }

    // not found
    throw std::out_of_range(f_name);
}

bool_type::bool_type(core &cr) : type(cr, cr, BOOL_KEYWORD, true) {}
bool_type::~bool_type() {}

int_type::int_type(core &cr) : type(cr, cr, INT_KEYWORD, true) {}
int_type::~int_type() {}

real_type::real_type(core &cr) : type(cr, cr, REAL_KEYWORD, true) {}
real_type::~real_type() {}

string_type::string_type(core &cr) : type(cr, cr, STRING_KEYWORD, true) {}
string_type::~string_type() {}
} // namespace ratio
