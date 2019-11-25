#include "row.h"
#include "assertion.h"
#include "lra_theory.h"
#include "sat_core.h"
#include <cassert>

namespace smt
{

row::row(lra_theory &th, const var x, lin l) : th(th), x(x), l(l) {}
row::~row() {}

bool row::propagate_lb(const var &v, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    // we make room for the first literal..
    cnfl.push_back(lit());
    if (l.vars.at(v).is_positive())
    {
        inf_rational lb(0);
        for (const auto &term : l.vars)
            if (term.second.is_positive())
                if (th.lb(term.first).is_negative_infinite())
                {
                    // nothing to propagate..
                    cnfl.clear();
                    return true;
                }
                else
                {
                    lb += term.second * th.lb(term.first);
                    cnfl.push_back(!*th.bounds.at(lra_theory::lb_index(term.first)).reason);
                }
            else if (term.second.is_negative())
                if (th.ub(term.first).is_positive_infinite())
                {
                    // nothing to propagate..
                    cnfl.clear();
                    return true;
                }
                else
                {
                    lb += term.second * th.ub(term.first);
                    cnfl.push_back(!*th.bounds.at(lra_theory::ub_index(term.first)).reason);
                }

        if (lb > th.lb(x))
            for (const auto &c : th.a_watches.at(x))
                if (lb > c->v)
                    switch (c->o)
                    {
                    case leq: // the assertion is unsatisfable..
                        cnfl[0] = lit(c->b, false);
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
            if (term.second.is_positive())
                if (th.ub(term.first).is_positive_infinite())
                {
                    // nothing to propagate..
                    cnfl.clear();
                    return true;
                }
                else
                {
                    ub += term.second * th.ub(term.first);
                    cnfl.push_back(!*th.bounds.at(lra_theory::ub_index(term.first)).reason);
                }
            else if (term.second.is_negative())
                if (th.lb(term.first).is_negative_infinite())
                {
                    // nothing to propagate..
                    cnfl.clear();
                    return true;
                }
                else
                {
                    ub += term.second * th.lb(term.first);
                    cnfl.push_back(!*th.bounds.at(lra_theory::lb_index(term.first)).reason);
                }

        if (ub < th.ub(x))
            for (const auto &c : th.a_watches.at(x))
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
                        cnfl[0] = lit(c->b, false);
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

bool row::propagate_ub(const var &v, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    // we make room for the first literal..
    cnfl.push_back(lit());
    if (l.vars.at(v).is_positive())
    {
        inf_rational ub(0);
        for (const auto &term : l.vars)
            if (term.second.is_positive())
                if (th.ub(term.first).is_positive_infinite())
                {
                    // nothing to propagate..
                    cnfl.clear();
                    return true;
                }
                else
                {
                    ub += term.second * th.ub(term.first);
                    cnfl.push_back(!*th.bounds.at(lra_theory::ub_index(term.first)).reason);
                }
            else if (term.second.is_negative())
                if (th.lb(term.first).is_negative_infinite())
                {
                    // nothing to propagate..
                    cnfl.clear();
                    return true;
                }
                else
                {
                    ub += term.second * th.lb(term.first);
                    cnfl.push_back(!*th.bounds.at(lra_theory::lb_index(term.first)).reason);
                }

        if (ub < th.ub(x))
            for (const auto &c : th.a_watches.at(x))
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
                        cnfl[0] = lit(c->b, false);
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
            if (term.second.is_positive())
                if (th.lb(term.first).is_negative_infinite())
                {
                    // nothing to propagate..
                    cnfl.clear();
                    return true;
                }
                else
                {
                    lb += term.second * th.lb(term.first);
                    cnfl.push_back(!*th.bounds.at(lra_theory::lb_index(term.first)).reason);
                }
            else if (term.second.is_negative())
                if (th.ub(term.first).is_positive_infinite())
                {
                    // nothing to propagate..
                    cnfl.clear();
                    return true;
                }
                else
                {
                    lb += term.second * th.ub(term.first);
                    cnfl.push_back(!*th.bounds.at(lra_theory::ub_index(term.first)).reason);
                }

        if (lb > th.lb(x))
            for (const auto &c : th.a_watches.at(x))
                if (lb > c->v)
                    switch (c->o)
                    {
                    case leq: // the assertion is unsatisfable..
                        cnfl[0] = lit(c->b, false);
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

std::string row::to_string() const { return "{ \"var\" : \"x" + std::to_string(x) + "\", \"expr\" : \"" + l.to_string() + "\" }"; }
} // namespace smt