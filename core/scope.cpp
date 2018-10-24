#include "scope.h"
#include "field.h"

namespace ratio
{

scope::scope(core &cr, scope &scp) : cr(cr), scp(scp) {}

scope::~scope()
{
    for (const auto &f : fields)
        delete f.second;
}

field &scope::get_field(const std::string &name) const
{
    const auto at_f = fields.find(name);
    if (at_f != fields.end())
        return *at_f->second;

    // if not here, check any enclosing scope
    return scp.get_field(name);
}

std::map<std::string, field *> scope::get_fields() const noexcept { return fields; }
} // namespace ratio
