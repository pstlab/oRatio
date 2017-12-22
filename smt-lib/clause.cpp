#include "clause.h"
#include "sat_core.h"

namespace smt
{

clause::clause(sat_core &s, const std::vector<lit> &lits) : s(s), lits(lits)
{
    s.watches[sat_core::index(!lits[0])].push_back(this);
    s.watches[sat_core::index(!lits[1])].push_back(this);
}

clause::~clause() {}

bool clause::propagate(const lit &p)
{
    // make sure false literal is lits[1]..
    if (lits[0].v == p.v)
        std::iter_swap(lits.begin(), lits.begin() + 1);

    // if 0th watch is true, the clause is already satisfied..
    if (s.value(lits[0]) == True)
    {
        s.watches[sat_core::index(p)].push_back(this);
        return true;
    }

    // we look for a new literal to watch..
    for (size_t i = 1; i < lits.size(); i++)
        if (s.value(lits.at(i)) != False)
        {
            std::iter_swap(lits.begin() + 1, lits.begin() + i);
            s.watches[sat_core::index(!lits.at(1))].push_back(this);
            return true;
        }

    // clause is unit under assignment..
    s.watches[sat_core::index(p)].push_back(this);
    return s.enqueue(lits[0], this);
}
}