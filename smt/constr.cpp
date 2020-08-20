#include "constr.h"
#include "sat_core.h"

namespace smt
{
    std::vector<constr *> &constr::watches(const lit &p) { return s.watches[index(p)]; }
    bool constr::enqueue(const lit &p) { return s.enqueue(p, this); }

    lbool constr::value(const var &x) const { return s.value(x); }
    lbool constr::value(const lit &p) const { return s.value(p); }
} // namespace smt