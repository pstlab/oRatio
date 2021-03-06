#include "clause.h"
#include "sat_core.h"
#include <algorithm>

namespace smt
{

clause::clause(sat_core &s, const std::vector<lit> &lits) : s(s), lits(lits)
{
    s.watches[sat_core::index(!lits[0])].push_back(this);
    s.watches[sat_core::index(!lits[1])].push_back(this);
}

clause::~clause() {}

const bool clause::propagate(const lit &p)
{
    // make sure false literal is lits[1]..
    if (lits[0].get_var() == p.get_var())
        std::swap(*(lits.begin()), *(lits.begin() + 1));

    // if 0th watch is true, the clause is already satisfied..
    if (s.value(lits[0]) == True)
    {
        s.watches[sat_core::index(p)].push_back(this);
        return true;
    }

    // we look for a new literal to watch..
    for (size_t i = 1; i < lits.size(); ++i)
        if (s.value(lits[i]) != False)
        {
            std::swap(*(lits.begin() + 1), *(lits.begin() + i));
            s.watches[sat_core::index(!lits[1])].push_back(this);
            return true;
        }

    // clause is unit under assignment..
    s.watches[sat_core::index(p)].push_back(this);
    return s.enqueue(lits[0], this);
}

const bool clause::simplify()
{
    size_t j = 0;
    for (size_t i = 0; i < lits.size(); ++i)
        switch (s.value(lits[i]))
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
    auto &l0_w = s.watches[sat_core::index(!lits[0])];
    l0_w.erase(std::find(l0_w.begin(), l0_w.end(), this));
    auto &l1_w = s.watches[sat_core::index(!lits[1])];
    l1_w.erase(std::find(l1_w.begin(), l1_w.end(), this));
    delete this;
}
} // namespace smt
