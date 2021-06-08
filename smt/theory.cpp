#include "theory.h"
#include "sat_core.h"
#include "clause.h"

namespace smt
{
    SMT_EXPORT theory::theory(sat_core &sat) : sat(sat) { sat.add_theory(*this); }
    SMT_EXPORT theory::~theory() {}

    SMT_EXPORT void theory::bind(const var &v) noexcept { sat.bind(v, *this); }

    SMT_EXPORT void theory::swap_conflict(theory &th) noexcept { std::swap(cnfl, th.cnfl); }

    SMT_EXPORT bool theory::backtrack_analyze_and_backjump() noexcept
    {
        size_t bt_level = 0;
        for (const auto &l : cnfl)
            if (bt_level < sat.level[variable(l)])
                bt_level = sat.level[variable(l)];

        while (sat.decision_level() > bt_level)
            sat.pop();

        if (sat.root_level())
            return false;

        analyze_and_backjump();
        return true;
    }

    void theory::analyze_and_backjump() noexcept
    {
        // we create a conflict clause for the analysis..
        std::vector<lit> c_cnfl;
        std::swap(c_cnfl, cnfl);
        clause cnfl_cl(sat, c_cnfl);

        // .. and we analyze the conflict..
        std::vector<lit> no_good;
        size_t bt_level;
        sat.analyze(cnfl_cl, no_good, bt_level);

        // we backjump..
        while (sat.decision_level() > bt_level)
            sat.pop();
        // .. and record the no-good..
        sat.record(no_good);
    }
    SMT_EXPORT void theory::record(const std::vector<lit> &cls) noexcept { sat.record(cls); }
} // namespace smt