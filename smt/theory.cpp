#include "theory.h"
#include "sat_core.h"

namespace smt
{
    theory::theory(sat_core &sat) : sat(sat) { sat.add_theory(*this); }
    theory::~theory() {}

    void theory::bind(const var &v) noexcept { sat.bind(v, *this); }
    void theory::record(const std::vector<lit> &cls) noexcept { sat.record(cls); }
} // namespace smt