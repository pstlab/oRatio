#include "lra_constraint.h"
#include "lra_theory.h"
#include "sat_core.h"
#include <cassert>

namespace smt
{

    assertion::assertion(lra_theory &th, const op o, const lit b, const var x, const inf_rational &v) : th(th), o(o), b(b), x(x), v(v) { th.a_watches[x].push_back(this); }
    assertion::~assertion() {}

    bool assertion::propagate_lb(const var &x_i)
    {
        assert(cnfl.empty());
        if (th.lb(x_i) > v)
            switch (o)
            {
            case leq: // the assertion is unsatisfable: [x_i >= lb(x_i)] -> ![x_i <= v]..
                switch (th.sat.value(b))
                {
                case True:                                                          // we have a propositional inconsistency..
                    cnfl.push_back(!b);                                             // either the literal 'b' is false ..
                    cnfl.push_back(!th.c_bounds[lra_theory::lb_index(x_i)].reason); // or what asserted the lower bound is false..
                    return false;
                case False: // nothing to propagate..
                    break;
                case Undefined: // we propagate information to the sat core..
                    th.record({!b, !th.c_bounds[lra_theory::lb_index(x_i)].reason});
                    break;
                }
                break;
            case geq: // the assertion is satisfied; [x_i >= lb(x_i)] -> [x_i >= v]..
                switch (th.sat.value(b))
                {
                case True: // nothing to propagate..
                    break;
                case False:                                                         // we have a propositional inconsistency..
                    cnfl.push_back(b);                                              // either the literal 'b' is true ..
                    cnfl.push_back(!th.c_bounds[lra_theory::lb_index(x_i)].reason); // or what asserted the lower bound is false..
                    return false;
                case Undefined: // we propagate information to the sat core..
                    th.record({b, !th.c_bounds[lra_theory::lb_index(x_i)].reason});
                    break;
                }
                break;
            }

        return true;
    }

    bool assertion::propagate_ub(const var &x_i)
    {
        assert(cnfl.empty());
        if (th.ub(x_i) < v)
            switch (o)
            {
            case leq: // the assertion is satisfied: [x_i <= ub(x_i)] -> [x_i <= v]..
                switch (th.sat.value(b))
                {
                case True: // nothing to propagate..
                    break;
                case False:                                                         // we have a propositional inconsistency..
                    cnfl.push_back(b);                                              // either the literal 'b' is true ..
                    cnfl.push_back(!th.c_bounds[lra_theory::ub_index(x_i)].reason); // or what asserted the upper bound is false..
                    return false;
                case Undefined: // we propagate information to the sat core..
                    th.record({b, !th.c_bounds[lra_theory::ub_index(x_i)].reason});
                    break;
                }
                break;
            case geq: // the assertion is unsatisfable; [x_i <= ub(x_i)] -> ![x_i >= v]..
                switch (th.sat.value(b))
                {
                case True:                                                          // we have a propositional inconsistency..
                    cnfl.push_back(!b);                                             // either the literal 'b' is false ..
                    cnfl.push_back(!th.c_bounds[lra_theory::ub_index(x_i)].reason); // or what asserted the upper bound is false..
                    return false;
                case False: // nothing to propagate..
                    break;
                case Undefined: // we propagate information to the sat core..
                    th.record({!b, !th.c_bounds[lra_theory::ub_index(x_i)].reason});
                    break;
                }
                break;
            }

        return true;
    }

    std::string to_string(const assertion &a)
    {
        std::string asrt;
        asrt += "{ \"lit\" : \"" + to_string(a.b) + "\", \"val\" : \"";
        switch (a.th.get_core().value(a.b))
        {
        case True:
            asrt += "T";
            break;
        case False:
            asrt += "F";
            break;
        case Undefined:
            asrt += "U";
            break;
        }
        asrt += "\", \"constr\" : \"x" + std::to_string(a.x);
        switch (a.o)
        {
        case leq:
            asrt += " <= ";
            break;
        case geq:
            asrt += " >= ";
            break;
        }
        asrt += to_string(a.v) + "\" }";
        return asrt;
    }

    row::row(lra_theory &th, const var x, lin l) : th(th), x(x), l(l) {}
    row::~row() {}

    bool row::propagate_lb(const var &v)
    {
        assert(cnfl.empty());
        // we make room for the first literal..
        cnfl.push_back(lit());
        if (is_positive(l.vars.at(v)))
        {
            inf_rational lb(0);
            for (const auto &term : l.vars)
                if (is_positive(term.second))
                    if (is_negative_infinite(th.lb(term.first)))
                    { // nothing to propagate..
                        cnfl.clear();
                        return true;
                    }
                    else
                    {
                        lb += term.second * th.lb(term.first);
                        cnfl.push_back(!th.c_bounds[lra_theory::lb_index(term.first)].reason);
                    }
                else if (is_negative(term.second))
                    if (is_positive_infinite(th.ub(term.first)))
                    { // nothing to propagate..
                        cnfl.clear();
                        return true;
                    }
                    else
                    {
                        lb += term.second * th.ub(term.first);
                        cnfl.push_back(!th.c_bounds[lra_theory::ub_index(term.first)].reason);
                    }

            if (lb > th.lb(x))
                for (const auto &c : th.a_watches[x])
                    if (lb > c->v)
                        switch (c->o)
                        {
                        case leq: // the assertion is unsatisfable..
                            cnfl[0] = !c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // we have a propositional inconsistency..
                                return false;
                            case False: // nothing to propagate..
                                break;
                            case Undefined: // we propagate information to the sat core..
                                th.record(cnfl);
                                break;
                            }
                            break;
                        case geq: // the assertion is satisfied..
                            cnfl[0] = c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // nothing to propagate..
                                break;
                            case False: // we have a propositional inconsistency..
                                return false;
                            case Undefined: // we propagate information to the sat core..
                                th.record(cnfl);
                                break;
                            }
                            break;
                        }
        }
        else
        {
            inf_rational ub(0);
            for (const auto &term : l.vars)
                if (is_positive(term.second))
                    if (is_positive_infinite(th.ub(term.first)))
                    { // nothing to propagate..
                        cnfl.clear();
                        return true;
                    }
                    else
                    {
                        ub += term.second * th.ub(term.first);
                        cnfl.push_back(!th.c_bounds[lra_theory::ub_index(term.first)].reason);
                    }
                else if (is_negative(term.second))
                    if (is_negative_infinite(th.lb(term.first)))
                    { // nothing to propagate..
                        cnfl.clear();
                        return true;
                    }
                    else
                    {
                        ub += term.second * th.lb(term.first);
                        cnfl.push_back(!th.c_bounds[lra_theory::lb_index(term.first)].reason);
                    }

            if (ub < th.ub(x))
                for (const auto &c : th.a_watches[x])
                    if (ub < c->v)
                        switch (c->o)
                        {
                        case leq: // the assertion is satisfied..
                            cnfl[0] = c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // nothing to propagate..
                                break;
                            case False: // we have a propositional inconsistency..
                                return false;
                            case Undefined: // we propagate information to the sat core..
                                th.record(cnfl);
                                break;
                            }
                            break;
                        case geq: // the assertion is unsatisfable..
                            cnfl[0] = !c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // we have a propositional inconsistency..
                                return false;
                            case False: // nothing to propagate..
                                break;
                            case Undefined: // we propagate information to the sat core..
                                th.record(cnfl);
                                break;
                            }
                            break;
                        }
        }

        cnfl.clear();
        return true;
    }

    bool row::propagate_ub(const var &v)
    {
        assert(cnfl.empty());
        // we make room for the first literal..
        cnfl.push_back(lit());
        if (is_positive(l.vars.at(v)))
        {
            inf_rational ub(0);
            for (const auto &term : l.vars)
                if (is_positive(term.second))
                    if (is_positive_infinite(th.ub(term.first)))
                    { // nothing to propagate..
                        cnfl.clear();
                        return true;
                    }
                    else
                    {
                        ub += term.second * th.ub(term.first);
                        cnfl.push_back(!th.c_bounds[lra_theory::ub_index(term.first)].reason);
                    }
                else if (is_negative(term.second))
                    if (is_negative_infinite(th.lb(term.first)))
                    { // nothing to propagate..
                        cnfl.clear();
                        return true;
                    }
                    else
                    {
                        ub += term.second * th.lb(term.first);
                        cnfl.push_back(!th.c_bounds[lra_theory::lb_index(term.first)].reason);
                    }

            if (ub < th.ub(x))
                for (const auto &c : th.a_watches[x])
                    if (ub < c->v)
                        switch (c->o)
                        {
                        case leq: // the assertion is satisfied..
                            cnfl[0] = c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // nothing to propagate..
                                break;
                            case False: // we have a propositional inconsistency..
                                return false;
                            case Undefined: // we propagate information to the sat core..
                                th.record(cnfl);
                                break;
                            }
                            break;
                        case geq: // the assertion is unsatisfable..
                            cnfl[0] = !c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // we have a propositional inconsistency..
                                return false;
                            case False: // nothing to propagate..
                                break;
                            case Undefined: // we propagate information to the sat core..
                                th.record(cnfl);
                                break;
                            }
                            break;
                        }
        }
        else
        {
            inf_rational lb(0);
            for (const auto &term : l.vars)
                if (is_positive(term.second))
                    if (is_negative_infinite(th.lb(term.first)))
                    { // nothing to propagate..
                        cnfl.clear();
                        return true;
                    }
                    else
                    {
                        lb += term.second * th.lb(term.first);
                        cnfl.push_back(!th.c_bounds[lra_theory::lb_index(term.first)].reason);
                    }
                else if (is_negative(term.second))
                    if (is_positive_infinite(th.ub(term.first)))
                    { // nothing to propagate..
                        cnfl.clear();
                        return true;
                    }
                    else
                    {
                        lb += term.second * th.ub(term.first);
                        cnfl.push_back(!th.c_bounds[lra_theory::ub_index(term.first)].reason);
                    }

            if (lb > th.lb(x))
                for (const auto &c : th.a_watches[x])
                    if (lb > c->v)
                        switch (c->o)
                        {
                        case leq: // the assertion is unsatisfable..
                            cnfl[0] = !c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // we have a propositional inconsistency..
                                return false;
                            case False: // nothing to propagate..
                                break;
                            case Undefined: // we propagate information to the sat core..
                                th.record(cnfl);
                                break;
                            }
                            break;
                        case geq: // the assertion is satisfied..
                            cnfl[0] = c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // nothing to propagate..
                                break;
                            case False: // we have a propositional inconsistency..
                                return false;
                            case Undefined: // we propagate information to the sat core..
                                th.record(cnfl);
                                break;
                            }
                            break;
                        }
        }

        cnfl.clear();
        return true;
    }

    std::string to_string(const row &r) { return "{ \"var\" : \"x" + std::to_string(r.x) + "\", \"expr\" : \"" + to_string(r.l) + "\" }"; }
} // namespace smt