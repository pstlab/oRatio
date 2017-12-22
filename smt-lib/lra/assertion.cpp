#include "assertion.h"
#include "lra_theory.h"
#include "sat_core.h"
#include <cassert>

namespace smt
{

assertion::assertion(lra_theory &th, const op o, const var b, const var x, const inf_rational &v) : th(th), o(o), b(b), x(x), v(v) { th.a_watches[x].push_back(this); }
assertion::~assertion() {}

bool assertion::propagate_lb(const var &x_i, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    if (th.lb(x_i) > v)
        switch (o)
        {
        case leq: // the assertion is unsatisfable: [x_i >= lb(x_i)] -> ![x_i <= v]..
            switch (th.sat.value(b))
            {
            case True:                                                             // we have a propositional inconsistency..
                cnfl.push_back(lit(b, false));                                     // either the literal 'b' is false ..
                cnfl.push_back(!*th.assigns.at(lra_theory::lb_index(x_i)).reason); // or what asserted the lower bound is false..
                return false;
            case Undefined: // we propagate information to the sat core..
                th.record({lit(b, false), !*th.assigns.at(lra_theory::lb_index(x_i)).reason});
            }
            break;
        case geq: // the assertion is satisfied; [x_i >= lb(x_i)] -> [x_i >= v]..
            switch (th.sat.value(b))
            {
            case False:                                                            // we have a propositional inconsistency..
                cnfl.push_back(b);                                                 // either the literal 'b' is true ..
                cnfl.push_back(!*th.assigns.at(lra_theory::lb_index(x_i)).reason); // or what asserted the lower bound is false..
                return false;
            case Undefined: // we propagate information to the sat core..
                th.record({b, !*th.assigns.at(lra_theory::lb_index(x_i)).reason});
            }
            break;
        }

    return true;
}

bool assertion::propagate_ub(const var &x_i, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    if (th.ub(x_i) < v)
        switch (o)
        {
        case leq: // the assertion is satisfied: [x_i <= ub(x_i)] -> [x_i <= v]..
            switch (th.sat.value(b))
            {
            case False:                                                            // we have a propositional inconsistency..
                cnfl.push_back(b);                                                 // either the literal 'b' is true ..
                cnfl.push_back(!*th.assigns.at(lra_theory::ub_index(x_i)).reason); // or what asserted the upper bound is false..
                return false;
            case Undefined: // we propagate information to the sat core..
                th.record({b, !*th.assigns.at(lra_theory::ub_index(x_i)).reason});
            }
            break;
        case geq: // the assertion is unsatisfable; [x_i <= ub(x_i)] -> ![x_i >= v]..
            switch (th.sat.value(b))
            {
            case True:                                                             // we have a propositional inconsistency..
                cnfl.push_back(lit(b, false));                                     // either the literal 'b' is false ..
                cnfl.push_back(!*th.assigns.at(lra_theory::ub_index(x_i)).reason); // or what asserted the upper bound is false..
                return false;
            case Undefined: // we propagate information to the sat core..
                th.record({lit(b, false), !*th.assigns.at(lra_theory::ub_index(x_i)).reason});
            }
            break;
        }

    return true;
}
}