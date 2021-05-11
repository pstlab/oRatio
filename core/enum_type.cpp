#include "enum_type.h"
#include "core.h"

namespace ratio
{
    enum_type::enum_type(core &cr, scope &scp, std::string name) : type(cr, scp, name) {}
    enum_type::~enum_type() {}

    expr enum_type::new_instance(context &) { return get_core().new_enum(*this, get_all_instances()); }

    std::vector<item *> enum_type::get_all_instances() const noexcept
    {
        std::vector<item *> c_instances;
        for (const auto &i : instances)
            c_instances.push_back(&*i);

        for (const auto &es : enums)
        {
            std::vector<item *> es_instances = es->get_all_instances();
            c_instances.reserve(c_instances.size() + es_instances.size());
            c_instances.insert(c_instances.cend(), es_instances.cbegin(), es_instances.cend());
        }
        return c_instances;
    }
} // namespace ratio