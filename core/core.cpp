#include "core.h"

namespace ratio
{

core::core() : scope(*this, *this), env(*this), sat_cr(), lra_th(sat_cr), ov_th(sat_cr) {}

core::~core() {}

field &core::get_field(const std::string &name) const
{
    const auto at_f = fields.find(name);
    if (at_f != fields.end())
        return *at_f->second;

    // not found
    throw std::out_of_range(name);
}
} // namespace ratio
