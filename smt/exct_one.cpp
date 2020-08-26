#include "exct_one.h"
#include "sat_core.h"
#include <algorithm>
#include <cassert>

namespace smt
{

    exct_one::exct_one(sat_core &s, const lit &ctr, const std::vector<lit> &lits) : constr(s), ctr(ctr), lits(lits)
    {
        watches(!ctr).push_back(this);           // if ctr becomes false, either all lits must become false or at least two literals must become true..
        watches(ctr).push_back(this);            // if ctr becomes true, exactly one lit must become true..
        watches(!lits[0]).push_back(this);       // if all lits become false or at least two lits become true, ctr must become false; if all lits become false except one that becomes true, ctr must become true..
        for (size_t i = 0; i < lits.size(); ++i) // if any lit becomes true and ctr is true all the other lits must become false..
            watches(lits[i]).push_back(this);
    }

    exct_one::~exct_one() {}

    const bool exct_one::propagate(const lit &p)
    {
        if (variable(ctr) == variable(p))
        { // the control literal has been assigned..
            watches(p).push_back(this);
            size_t n_false = 0;
            lit undef_lit;
            if (value(ctr) == True)
            { // the control literal has become true..
                for (std::vector<lit>::const_iterator it = lits.begin(); it != lits.end(); ++it)
                    switch (value(*it))
                    {
                    case True: // one literal is already true, all the others must be false..
                        return std::all_of(lits.begin(), lits.end(), [this, it](const lit &lt) { return lt == *it || enqueue(!lt); });
                    case False:
                        n_false++;
                        break;
                    case Undefined:
                        undef_lit = *it;
                        break;
                    }
                if (n_false == lits.size() - 1) // only one literal is left undefined: we propagate..
                    return enqueue(undef_lit);
                assert(!std::all_of(lits.begin(), lits.end(), [this](const lit &lt) { return value(lt) == False; }));
                return true;
            }
            // the control literal has become false..
            bool found_true = false;
            for (std::vector<lit>::const_iterator it = lits.begin(); it != lits.end(); ++it)
                switch (value(*it))
                {
                case True:
                    if (found_true)
                        return true; // the exact-one constraint is already unsatisfiable (more then one lit is true)..
                    else
                        found_true = true;
                    break;
                case False:
                    n_false++;
                    break;
                case Undefined:
                    if (is_undefined(undef_lit))
                        undef_lit = *it;
                    else
                        return true;
                    break;
                }
            if (n_false == lits.size() - 1) // only one literal is left undefined: we propagate..
                return enqueue(!undef_lit);
            if (n_false == lits.size() - 2 && found_true)
                return enqueue(undef_lit);
            return true;
        }

        if (value(p) == True)
        { // a literal has become true..
            watches(p).push_back(this);
            switch (value(ctr))
            {
            case True: // the control literal is true, all literals except p must be false..
                return std::all_of(lits.begin(), lits.end(), [this, p](const lit &lt) { return lt == p || enqueue(!lt); });
            case False:
            { // the control literal is false, another literal must be true..
                size_t n_false = 0;
                lit undef_lit;
                for (std::vector<lit>::const_iterator it = lits.begin(); it != lits.end(); ++it)
                    if (*it != p)
                        switch (value(*it))
                        {
                        case True:
                            return true;
                        case False:
                            n_false++;
                            break;
                        case Undefined:
                            if (is_undefined(undef_lit))
                                undef_lit = *it;
                            else
                                return true;
                            break;
                        }
                if (n_false == lits.size() - 1) // only one literal is left undefined: we propagate..
                    return enqueue(undef_lit);
                return true;
            }
            case Undefined:
            {
                bool found_true = false;
                for (std::vector<lit>::const_iterator it = lits.begin(); it != lits.end(); ++it)
                    if (*it != p)
                        switch (value(*it))
                        {
                        case True:
                            if (found_true)
                                return enqueue(!ctr); // the exact-one constraint is already unsatisfiable (more then one lit is true)..
                            else
                                found_true = true;
                            break;
                        case Undefined:
                            return true;
                        }
                if (found_true)
                    return enqueue(ctr); // the exact-one constraint is already satisfied..
                else
                    return enqueue(!ctr); // the exact-one constraint is already unsatisfiable (all lits are false)..
            }
            break;
            }
        }

        // lits[0] has become false..
        switch (value(ctr))
        {
        case True:
        { // the control literal is true, all literals except one must be false..
            size_t n_false = 0;
            for (size_t i = 1; i < lits.size(); ++i)
                switch (value(lits[i]))
                {
                case False:
                    n_false++;
                    break;
                case Undefined:
                    if (is_undefined(lits[0]))
                        return true;
                    else
                    {
                        std::swap(*(lits.begin()), *(lits.begin() + i));
                        watches(!lits[0]).push_back(this);
                    }
                }
            if (n_false == lits.size() - 1) // only one literal is left undefined: we propagate..
                return enqueue(lits[0]);
            return true;
        }
        case False:
        {
            size_t n_false = 0;
            for (size_t i = 1; i < lits.size(); ++i)
                switch (value(lits[i]))
                {
                case True:
                    assert(std::any_of(lits.begin() + i + 1, lits.end(), [this](const lit &lt) { return value(lt) == True; }));
                    for (size_t j = i + 1; j < lits.size(); ++j)
                        if (value(lits[i]) == Undefined)
                        {
                            std::swap(*(lits.begin()), *(lits.begin() + i));
                            watches(!lits[0]).push_back(this);
                            return true; // the exact-one constraint is already unsatisfiable (more then one lit is true)..
                        }
                    break;
                case False:
                    n_false++;
                    break;
                case Undefined:
                    if (is_undefined(lits[0]))
                        return true;
                    else
                    {
                        std::swap(*(lits.begin()), *(lits.begin() + i));
                        watches(!lits[0]).push_back(this);
                        return true;
                    }
                    break;
                }
            if (n_false == lits.size() - 1) // only one literal is left undefined: we propagate..
                return enqueue(!lits[0]);
            if (n_false == lits.size() - 2 && found_true)
                return enqueue(lits[0]);
            return true;
        }
        break;
        case Undefined:
            break;
        }
        return true;

        return enqueue(!ctr);
    }

    const bool exct_one::simplify()
    {
        switch (value(ctr))
        {
        case True:
        case False:
            return true;
        default:
            size_t j = 0;
            for (size_t i = 0; i < lits.size(); ++i)
                if (value(lits[i]) == Undefined)
                    lits[j++] = lits[i];
            lits.resize(j);
            return false;
        }
    }

    void exct_one::remove()
    {
        delete this;
    }

    void exct_one::get_reason(const lit &p, std::vector<lit> &out_reason) const
    {
        assert(value(ctr) != Undefined);
        if (is_undefined(p))
        { // this exact-one is the conflicting constraint..
        }
    }
} // namespace smt