#include "constr.h"
#include "sat_core.h"

namespace smt
{
    constr::constr(sat_core &s) : s(s), id(s.constrs.size()) {}
    constr::~constr() {}

    std::vector<constr *> &constr::watches(const lit &p) noexcept { return s.watches[index(p)]; }
    bool constr::enqueue(const lit &p) noexcept { return s.enqueue(p, this); }

    lbool constr::value(const var &x) const noexcept { return s.value(x); }
    lbool constr::value(const lit &p) const noexcept { return s.value(p); }
} // namespace smt