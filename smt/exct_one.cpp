#include "exct_one.h"
#include "sat_core.h"
#include <algorithm>
#include <cassert>

namespace smt
{

    exct_one::exct_one(sat_core &s, const lit &ctr, const std::vector<lit> &lits) : constr(s), ctr(ctr), lits(lits)
    {
        watches(!ctr).push_back(this);           // if ctr becomes false, either all lits must become false or at least two literals must become true..
        watches(ctr).push_back(this);            // if ctr becomes true, at exactly one lit must become true..
        watches(!lits[0]).push_back(this);       // if all lits become false or at least two lits become true, ctr must become false; if all lits become false except one that becomes true, ctr must become true..
        for (size_t i = 0; i < lits.size(); ++i) // if any lit becomes true and ctr is true all the other lits must become false..
            watches(lits[i]).push_back(this);
    }

    exct_one::~exct_one() {}

    const bool exct_one::propagate(const lit &p)
    {
        if (variable(ctr) == variable(p))
        { // the control literal has been assigned..

            if (value(ctr) == True) // the control literal has become true..
                return false;
            // the control literal has become false..
            return false;
        }

        if (value(p) == True)
        { // a literal has become true..

            if (value(ctr) == True) // the control is true, all literals except p must be false..
                return std::all_of(lits.begin(), lits.end(), [this, p](const lit &lt) { return lt == p || enqueue(!lt); });
            return enqueue(ctr);
        }

        // lits[0] has become false..
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