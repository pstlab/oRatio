#include "clause.h"
#include "sat_core.h"
#include <algorithm>
#include <cassert>

namespace smt
{

    clause::clause(sat_core &s, const std::vector<lit> &lits) : constr(s), lits(lits)
    {
        watches(!lits[0]).push_back(this);
        watches(!lits[1]).push_back(this);
    }

    clause::~clause() {}

    const bool clause::propagate(const lit &p)
    {
        // make sure false literal is lits[1]..
        if (variable(lits[0]) == variable(p))
            std::swap(*(lits.begin()), *(lits.begin() + 1));

        // if 0th watch is true, the clause is already satisfied..
        if (value(lits[0]) == True)
        {
            watches(p).push_back(this);
            return true;
        }

        // we look for a new literal to watch..
        for (size_t i = 1; i < lits.size(); ++i)
            if (value(lits[i]) != False)
            {
                std::swap(*(lits.begin() + 1), *(lits.begin() + i));
                watches(!lits[1]).push_back(this);
                return true;
            }

        // clause is unit under assignment..
        watches(p).push_back(this);
        return enqueue(lits[0]);
    }

    const bool clause::simplify()
    {
        size_t j = 0;
        for (size_t i = 0; i < lits.size(); ++i)
            switch (value(lits[i]))
            {
            case True:
                return true;
            case Undefined:
                lits[j++] = lits[i];
                break;
            }
        lits.resize(j);
        return false;
    }

    void clause::remove()
    {
        auto &l0_w = watches(!lits[0]);
        l0_w.erase(std::find(l0_w.begin(), l0_w.end(), this));
        auto &l1_w = watches(!lits[1]);
        l1_w.erase(std::find(l1_w.begin(), l1_w.end(), this));
        delete this;
    }

    void clause::get_reason(const lit &p, std::vector<lit> &out_reason) const
    {
        assert(is_undefined(p) || p == lits[0]);
        out_reason.reserve(is_undefined(p) ? lits.size() : lits.size() - 1);
        for (size_t i = is_undefined(p) ? 0 : 1; i < lits.size(); ++i)
        {
            assert(value(lits[i]) == False);
            out_reason.push_back(!lits[i]);
        }
    }

    std::string to_string(const clause &c)
    {
        std::string cls;
        cls += "{ \"lits\" : [";

        for (std::vector<lit>::const_iterator it = c.lits.cbegin(); it != c.lits.cend(); ++it)
        {
            if (it != c.lits.cbegin())
                cls += ", ";
            cls += "{\"lit\" : \"" + to_string(*it) + "\", \"val\" : \"";
            switch (c.value(*it))
            {
            case True:
                cls += "T";
                break;
            case False:
                cls += "F";
                break;
            case Undefined:
                cls += "U";
                break;
            }
            cls += "\"}";
        }

        cls += +"] }";
        return cls;
    }
} // namespace smt