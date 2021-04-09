#include "theory.h"
#include "sat_core.h"

namespace smt
{
    SMT_EXPORT theory::theory(sat_core &sat) : sat(sat) { sat.add_theory(*this); }
    SMT_EXPORT theory::~theory() {}

    SMT_EXPORT void theory::bind(const var &v) noexcept { sat.bind(v, *this); }
    SMT_EXPORT void theory::record(const std::vector<lit> &cls) noexcept { sat.record(cls); }
} // namespace smt