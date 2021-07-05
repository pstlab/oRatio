#include "constr.h"
#include "sat_core.h"

namespace smt
{
    constr::constr(sat_core &s) : sat(s), id(s.constrs.size()) {}
    constr::~constr() {}

    std::vector<constr *> &constr::watches(const lit &p) noexcept { return sat.watches[index(p)]; }
    bool constr::enqueue(const lit &p) noexcept { return sat.enqueue(p, this); }

    lbool constr::value(const var &x) const noexcept { return sat.value(x); }
    lbool constr::value(const lit &p) const noexcept { return sat.value(p); }
} // namespace smt