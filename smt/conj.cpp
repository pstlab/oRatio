#include "conj.h"
#include "sat_core.h"
#include <algorithm>
#include <cassert>

namespace smt
{

    conj::conj(sat_core &s, const lit &ctr, const std::vector<lit> &lits) : constr(s), ctr(ctr), lits(lits)
    {
        watches(ctr).push_back(this);            // if ctr becomes true, all lits must become true..
        watches(!ctr).push_back(this);           // if ctr becomes false, at least one lit must be false..
        watches(lits[0]).push_back(this);        // if all lits become true, ctr must become true..
        for (size_t i = 0; i < lits.size(); ++i) // if any lit becomes false, ctr must become false..
            watches(!lits[i]).push_back(this);
    }

    conj::~conj() {}

    const bool conj::propagate(const lit &p)
    {
        if (variable(ctr) == variable(p))
        { // the control literal has been assigned..
            watches(p).push_back(this);
            if (value(ctr) == True) // the control literal has become true, so all the conjunction literals must be true..
                return std::all_of(lits.begin(), lits.end(), [this](const lit &lt) { return enqueue(lt); });
            // the control literal has become false, so at least one of the conjunction literals must be false..
            for (std::vector<lit>::iterator it = lits.begin(); it != lits.end(); ++it)
                switch (value(*it))
                {
                case False: // a conjunction literal is false so the conjunction is already negated..
                    return true;
                case Undefined:
                    if (std::all_of(it + 1, lits.end(), [this](const lit &lt) { return value(lt) == True; }))
                        // the conjunction is unit under assignment..
                        return enqueue(!*it);
                    else
                        return true;
                }
            return false;
        }

        if (value(p) == False)
        { // a literal has become false, so must be ctr..
            watches(p).push_back(this);
            return enqueue(!ctr);
        }

        // lits[0] has become true, we look for a new literal to watch..
        for (size_t i = 1; i < lits.size(); ++i)
            switch (value(lits[i]))
            {
            case False: // we have found, for a chance, a false literal..
                std::swap(*(lits.begin()), *(lits.begin() + i));
                watches(lits[0]).push_back(this);
                return enqueue(!ctr);
            case Undefined:
                std::swap(*(lits.begin()), *(lits.begin() + i));
                watches(lits[0]).push_back(this);
                return true;
            }

        // all the literals are true, so must be ctr..
        watches(p).push_back(this);
        return enqueue(ctr);
    }

    const bool conj::simplify()
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

    void conj::remove()
    {
        auto &l_ctr_w = watches(ctr);
        l_ctr_w.erase(std::find(l_ctr_w.begin(), l_ctr_w.end(), this));

        auto &l_nctr_w = watches(!ctr);
        l_nctr_w.erase(std::find(l_nctr_w.begin(), l_nctr_w.end(), this));

        auto &l0_w = watches(lits[0]);
        l0_w.erase(std::find(l0_w.begin(), l0_w.end(), this));

        for (size_t i = 0; i < lits.size(); ++i)
        {
            auto &li_w = watches(!lits[i]);
            li_w.erase(std::find(li_w.begin(), li_w.end(), this));
        }
        delete this;
    }

    void conj::get_reason(const lit &p, std::vector<lit> &out_reason) const
    {
        assert(value(ctr) != Undefined);
        if (is_undefined(p))
        { // this conjunction is the conflicting constraint..
            if (value(ctr) == True)
            {
                for (size_t i = 0; i < lits.size(); ++i)
                    if (value(lits[i]) == False)
                    {
                        out_reason.push_back(ctr);
                        out_reason.push_back(!lits[i]);
                    }
            }
            else
            {
                out_reason.reserve(lits.size() + 1);
                out_reason.push_back(!ctr);
                for (size_t i = 0; i < lits.size(); ++i)
                {
                    assert(value(lits[i]) == True);
                    out_reason.push_back(lits[i]);
                }
            }
        }
        else if (variable(ctr) == variable(p))
            if (value(ctr) == False)
            { // ctr has become false because a conjunction literal has become false..
                for (size_t i = 0; i < lits.size(); ++i)
                    if (value(lits[i]) == False)
                    {
                        out_reason.push_back(!lits[i]);
                        break;
                    }
            }
            else
            { // ctr has become true because all conjunction literals have become true..
                out_reason.reserve(lits.size());
                for (size_t i = 0; i < lits.size(); ++i)
                {
                    assert(value(lits[i]) == True);
                    out_reason.push_back(lits[i]);
                }
            }
        else
        {
            if (value(p) == True) // p has become true because ctr has become true..
                out_reason.push_back(ctr);
            else
            { // p has become false because ctr has become false and all the conjunction literals except p have become true..
                out_reason.reserve(lits.size() + 1);
                out_reason.push_back(!ctr);
                for (size_t i = 0; i < lits.size(); ++i)
                    if (lits[i] != p)
                    {
                        assert(value(lits[i]) == True);
                        out_reason.push_back(lits[i]);
                    }
            }
        }
    }
} // namespace smt