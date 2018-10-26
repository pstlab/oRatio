#include "scope.h"
#include "field.h"

namespace ratio
{

scope::scope(core &cr, scope &scp) : cr(cr), scp(scp) {}

scope::~scope()
{
    // we delete the fields defined within this scope..
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

type &scope::get_type(const std::string &name) const { return scp.get_type(name); }
std::map<std::string, type *> scope::get_types() const noexcept { return scp.get_types(); }

void scope::new_fields(scope &s, const std::vector<field *> &fs)
{
    for (const auto &f : fs)
        s.fields.insert({f->get_name(), f});
}
void scope::new_fields(const std::vector<field *> &fs)
{
    for (const auto &f : fs)
        fields.insert({f->get_name(), f});
}
} // namespace ratio
