#include "lra_constraint.h"
#include "lra_theory.h"
#include "sat_core.h"
#include <cassert>

namespace smt
{
    assertion::assertion(lra_theory &th, const op o, const lit b, const var x, const inf_rational &v) : th(th), o(o), b(b), x(x), v(v) { th.a_watches[x].push_back(this); }
    assertion::~assertion() {}

    bool assertion::propagate_lb(const var &x_i) noexcept
    {
        assert(th.cnfl.empty());
        if (th.lb(x_i) > v)
            switch (o)
            {
            case leq: // the assertion is unsatisfable: [x_i >= lb(x_i)] -> ![x_i <= v]..
                switch (th.sat.value(b))
                {
                case True:                                                             // we have a propositional inconsistency (notice that this can happen in case some propositional literal has been assigned but the theory did not propagate yet)..
                    th.cnfl.push_back(!b);                                             // either the literal 'b' is false ..
                    th.cnfl.push_back(!th.c_bounds[lra_theory::lb_index(x_i)].reason); // or what asserted the lower bound is false..
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
                case False:                                                            // we have a propositional inconsistency (notice that this can happen in case some propositional literal has been assigned but the theory did not propagate yet)..
                    th.cnfl.push_back(b);                                              // either the literal 'b' is true ..
                    th.cnfl.push_back(!th.c_bounds[lra_theory::lb_index(x_i)].reason); // or what asserted the lower bound is false..
                    return false;
                case Undefined: // we propagate information to the sat core..
                    th.record({b, !th.c_bounds[lra_theory::lb_index(x_i)].reason});
                    break;
                }
                break;
            }

        return true;
    }

    bool assertion::propagate_ub(const var &x_i) noexcept
    {
        assert(th.cnfl.empty());
        if (th.ub(x_i) < v)
            switch (o)
            {
            case leq: // the assertion is satisfied: [x_i <= ub(x_i)] -> [x_i <= v]..
                switch (th.sat.value(b))
                {
                case True: // nothing to propagate..
                    break;
                case False:                                                            // we have a propositional inconsistency (notice that this can happen in case some propositional literal has been assigned but the theory did not propagate yet)..
                    th.cnfl.push_back(b);                                              // either the literal 'b' is true ..
                    th.cnfl.push_back(!th.c_bounds[lra_theory::ub_index(x_i)].reason); // or what asserted the upper bound is false..
                    return false;
                case Undefined: // we propagate information to the sat core..
                    th.record({b, !th.c_bounds[lra_theory::ub_index(x_i)].reason});
                    break;
                }
                break;
            case geq: // the assertion is unsatisfable; [x_i <= ub(x_i)] -> ![x_i >= v]..
                switch (th.sat.value(b))
                {
                case True:                                                             // we have a propositional inconsistency (notice that this can happen in case some propositional literal has been assigned but the theory did not propagate yet)..
                    th.cnfl.push_back(!b);                                             // either the literal 'b' is false ..
                    th.cnfl.push_back(!th.c_bounds[lra_theory::ub_index(x_i)].reason); // or what asserted the upper bound is false..
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

    json assertion::to_json() const noexcept
    {
        json j_asrt;
        j_asrt->set("lit", new string_val(to_string(b)));
        switch (th.get_core().value(b))
        {
        case True:
            j_asrt->set("val", new string_val("T"));
            break;
        case False:
            j_asrt->set("val", new string_val("F"));
            break;
        case Undefined:
            j_asrt->set("val", new string_val("U"));
            break;
        }
        j_asrt->set("constr", new string_val("x" + std::to_string(x) + (o == geq ? " >= " : " <= ") + to_string(v)));
        return j_asrt;
    }

    row::row(lra_theory &th, const var x, lin l) : th(th), x(x), l(l) {}
    row::~row() {}

    bool row::propagate_lb(const var &v) noexcept
    {
        assert(th.cnfl.empty());
        // we make room for the first literal..
        th.cnfl.push_back(lit());
        if (is_positive(l.vars.at(v)))
        {
            inf_rational lb(0);
            for (const auto &[c_v, c] : l.vars)
                if (is_positive(c))
                    if (is_negative_infinite(th.lb(c_v)))
                    { // nothing to propagate..
                        th.cnfl.clear();
                        return true;
                    }
                    else
                    {
                        lb += c * th.lb(c_v);
                        th.cnfl.push_back(!th.c_bounds[lra_theory::lb_index(c_v)].reason);
                    }
                else if (is_negative(c))
                    if (is_positive_infinite(th.ub(c_v)))
                    { // nothing to propagate..
                        th.cnfl.clear();
                        return true;
                    }
                    else
                    {
                        lb += c * th.ub(c_v);
                        th.cnfl.push_back(!th.c_bounds[lra_theory::ub_index(c_v)].reason);
                    }

            if (lb > th.lb(x))
                for (const auto &c : th.a_watches[x])
                    if (lb > c->v)
                        switch (c->o)
                        {
                        case leq: // the assertion is unsatisfable..
                            th.cnfl[0] = !c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // we have a propositional inconsistency (notice that this can happen in case some propositional literal has been assigned but the theory did not propagate yet)..
                                return false;
                            case False: // nothing to propagate..
                                break;
                            case Undefined: // we propagate information to the sat core..
                                th.record(th.cnfl);
                                break;
                            }
                            break;
                        case geq: // the assertion is satisfied..
                            th.cnfl[0] = c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // nothing to propagate..
                                break;
                            case False: // we have a propositional inconsistency (notice that this can happen in case some propositional literal has been assigned but the theory did not propagate yet)..
                                return false;
                            case Undefined: // we propagate information to the sat core..
                                th.record(th.cnfl);
                                break;
                            }
                            break;
                        }
        }
        else
        {
            inf_rational ub(0);
            for (const auto &[c_v, c] : l.vars)
                if (is_positive(c))
                    if (is_positive_infinite(th.ub(c_v)))
                    { // nothing to propagate..
                        th.cnfl.clear();
                        return true;
                    }
                    else
                    {
                        ub += c * th.ub(c_v);
                        th.cnfl.push_back(!th.c_bounds[lra_theory::ub_index(c_v)].reason);
                    }
                else if (is_negative(c))
                    if (is_negative_infinite(th.lb(c_v)))
                    { // nothing to propagate..
                        th.cnfl.clear();
                        return true;
                    }
                    else
                    {
                        ub += c * th.lb(c_v);
                        th.cnfl.push_back(!th.c_bounds[lra_theory::lb_index(c_v)].reason);
                    }

            if (ub < th.ub(x))
                for (const auto &c : th.a_watches[x])
                    if (ub < c->v)
                        switch (c->o)
                        {
                        case leq: // the assertion is satisfied..
                            th.cnfl[0] = c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // nothing to propagate..
                                break;
                            case False: // we have a propositional inconsistency (notice that this can happen in case some propositional literal has been assigned but the theory did not propagate yet)..
                                return false;
                            case Undefined: // we propagate information to the sat core..
                                th.record(th.cnfl);
                                break;
                            }
                            break;
                        case geq: // the assertion is unsatisfable..
                            th.cnfl[0] = !c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // we have a propositional inconsistency (notice that this can happen in case some propositional literal has been assigned but the theory did not propagate yet)..
                                return false;
                            case False: // nothing to propagate..
                                break;
                            case Undefined: // we propagate information to the sat core..
                                th.record(th.cnfl);
                                break;
                            }
                            break;
                        }
        }

        th.cnfl.clear();
        return true;
    }

    bool row::propagate_ub(const var &v) noexcept
    {
        assert(th.cnfl.empty());
        // we make room for the first literal..
        th.cnfl.push_back(lit());
        if (is_positive(l.vars.at(v)))
        {
            inf_rational ub(0);
            for (const auto &[c_v, c] : l.vars)
                if (is_positive(c))
                    if (is_positive_infinite(th.ub(c_v)))
                    { // nothing to propagate..
                        th.cnfl.clear();
                        return true;
                    }
                    else
                    {
                        ub += c * th.ub(c_v);
                        th.cnfl.push_back(!th.c_bounds[lra_theory::ub_index(c_v)].reason);
                    }
                else if (is_negative(c))
                    if (is_negative_infinite(th.lb(v)))
                    { // nothing to propagate..
                        th.cnfl.clear();
                        return true;
                    }
                    else
                    {
                        ub += c * th.lb(c_v);
                        th.cnfl.push_back(!th.c_bounds[lra_theory::lb_index(c_v)].reason);
                    }

            if (ub < th.ub(x))
                for (const auto &c : th.a_watches[x])
                    if (ub < c->v)
                        switch (c->o)
                        {
                        case leq: // the assertion is satisfied..
                            th.cnfl[0] = c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // nothing to propagate..
                                break;
                            case False: // we have a propositional inconsistency (notice that this can happen in case some propositional literal has been assigned but the theory did not propagate yet)..
                                return false;
                            case Undefined: // we propagate information to the sat core..
                                th.record(th.cnfl);
                                break;
                            }
                            break;
                        case geq: // the assertion is unsatisfable..
                            th.cnfl[0] = !c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // we have a propositional inconsistency (notice that this can happen in case some propositional literal has been assigned but the theory did not propagate yet)..
                                return false;
                            case False: // nothing to propagate..
                                break;
                            case Undefined: // we propagate information to the sat core..
                                th.record(th.cnfl);
                                break;
                            }
                            break;
                        }
        }
        else
        {
            inf_rational lb(0);
            for (const auto &[c_v, c] : l.vars)
                if (is_positive(c))
                    if (is_negative_infinite(th.lb(c_v)))
                    { // nothing to propagate..
                        th.cnfl.clear();
                        return true;
                    }
                    else
                    {
                        lb += c * th.lb(c_v);
                        th.cnfl.push_back(!th.c_bounds[lra_theory::lb_index(c_v)].reason);
                    }
                else if (is_negative(c))
                    if (is_positive_infinite(th.ub(c_v)))
                    { // nothing to propagate..
                        th.cnfl.clear();
                        return true;
                    }
                    else
                    {
                        lb += c * th.ub(c_v);
                        th.cnfl.push_back(!th.c_bounds[lra_theory::ub_index(c_v)].reason);
                    }

            if (lb > th.lb(x))
                for (const auto &c : th.a_watches[x])
                    if (lb > c->v)
                        switch (c->o)
                        {
                        case leq: // the assertion is unsatisfable..
                            th.cnfl[0] = !c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // we have a propositional inconsistency (notice that this can happen in case some propositional literal has been assigned but the theory did not propagate yet)..
                                return false;
                            case False: // nothing to propagate..
                                break;
                            case Undefined: // we propagate information to the sat core..
                                th.record(th.cnfl);
                                break;
                            }
                            break;
                        case geq: // the assertion is satisfied..
                            th.cnfl[0] = c->b;
                            switch (th.sat.value(c->b))
                            {
                            case True: // nothing to propagate..
                                break;
                            case False: // we have a propositional inconsistency (notice that this can happen in case some propositional literal has been assigned but the theory did not propagate yet)..
                                return false;
                            case Undefined: // we propagate information to the sat core..
                                th.record(th.cnfl);
                                break;
                            }
                            break;
                        }
        }

        th.cnfl.clear();
        return true;
    }

    json row::to_json() const noexcept
    {
        json j_row;
        j_row->set("var", new string_val("x" + std::to_string(x)));
        j_row->set("expr", new string_val(to_string(l)));
        return j_row;
    }
} // namespace smt