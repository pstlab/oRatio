#include "enum_type.h"
#include "core.h"

namespace ratio
{

enum_type::enum_type(core &cr, scope &scp, std::string name) : type(cr, scp, name) {}
enum_type::~enum_type() {}

expr enum_type::new_instance(context &) { return get_core().new_enum(*this, get_all_instances()); }

std::unordered_set<item *> enum_type::get_all_instances() const
{
    std::unordered_set<item *> c_instances;
    for (const auto &i : instances)
        c_instances.insert(&*i);

    for (const auto &es : enums)
    {
        std::unordered_set<item *> es_instances = es->get_all_instances();
        c_instances.insert(es_instances.begin(), es_instances.end());
    }
    return c_instances;
}
}