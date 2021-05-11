#include "scope.h"
#include "field.h"

namespace ratio
{
    scope::scope(core &cr, scope &scp) : cr(cr), scp(scp) {}
    scope::~scope()
    {
        // we delete the fields defined within this scope..
        for (const auto &[f_name, f] : fields)
            delete f;
    }

    CORE_EXPORT const field &scope::get_field(const std::string &name) const
    {
        if (const auto at_f = fields.find(name); at_f != fields.cend())
            return *at_f->second;

        // if not here, check any enclosing scope
        return scp.get_field(name);
    }

    CORE_EXPORT std::map<std::string, const field *> scope::get_fields() const noexcept { return fields; }

    CORE_EXPORT const method &scope::get_method(const std::string &name, const std::vector<const type *> &ts) const { return scp.get_method(name, ts); }
    CORE_EXPORT std::vector<const method *> scope::get_methods() const noexcept { return scp.get_methods(); }

    CORE_EXPORT type &scope::get_type(const std::string &name) const { return scp.get_type(name); }
    CORE_EXPORT std::map<std::string, type *> scope::get_types() const noexcept { return scp.get_types(); }

    CORE_EXPORT predicate &scope::get_predicate(const std::string &name) const { return scp.get_predicate(name); }
    CORE_EXPORT std::map<std::string, predicate *> scope::get_predicates() const noexcept { return scp.get_predicates(); }

    CORE_EXPORT void scope::new_fields(scope &s, const std::vector<const field *> &fs)
    {
        for (const auto &f : fs)
            s.fields.emplace(f->get_name(), f);
    }
    void scope::new_fields(const std::vector<const field *> &fs)
    {
        for (const auto &f : fs)
            fields.emplace(f->get_name(), f);
    }
} // namespace ratio