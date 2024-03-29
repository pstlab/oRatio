#include "theory.h"
#include "sat_core.h"
#include "clause.h"
#include <algorithm>

namespace smt
{
    SMT_EXPORT theory::theory(sat_core &s) : sat(&s) { sat->theories.push_back(this); }
    SMT_EXPORT theory::~theory() { sat->theories.erase(std::find(sat->theories.cbegin(), sat->theories.cend(), this)); }

    SMT_EXPORT void theory::bind(const var &v) noexcept { sat->bind(v, *this); }

    SMT_EXPORT void theory::swap_conflict(theory &th) noexcept { std::swap(cnfl, th.cnfl); }

    SMT_EXPORT bool theory::backtrack_analyze_and_backjump() noexcept
    {
        // we backtrack to a level at which we can analyze the conflict..
        size_t bt_level = 0;
        for (const auto &l : cnfl)
            if (bt_level < sat->level[variable(l)])
                bt_level = sat->level[variable(l)];

        while (sat->decision_level() > bt_level)
            sat->pop();

        if (sat->root_level())
            return sat->new_clause(cnfl) && sat->propagate();

        // we analyze the conflict and backjump..
        analyze_and_backjump();
        return sat->propagate();
    }

    void theory::analyze_and_backjump() noexcept
    {
        // we create a conflict clause for the analysis..
        clause cnfl_cl(*sat, std::move(cnfl));

        // .. and we analyze the conflict..
        std::vector<lit> no_good;
        size_t bt_level;
        sat->analyze(cnfl_cl, no_good, bt_level);
        cnfl.clear();

        // we backjump..
        while (sat->decision_level() > bt_level)
            sat->pop();
        // .. and record the no-good..
        sat->record(no_good);
    }
    SMT_EXPORT void theory::record(std::vector<lit> cls) noexcept { sat->record(std::move(cls)); }
} // namespace smt