#include "core.h"

namespace ratio
{

core::core() : scope(*this, *this), env(*this), sat_cr(), lra_th(sat_cr), ov_th(sat_cr) {}

core::~core() {}
} // namespace ratio
