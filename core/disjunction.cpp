#include "disjunction.h"
#include "conjunction.h"

namespace ratio
{

    disjunction::disjunction(core &cr, scope &scp, const std::vector<const conjunction *> &conjs) : scope(cr, scp), conjunctions(conjs) {}
    disjunction::~disjunction()
    {
        for (const auto &c : conjunctions)
            delete c;
    }
} // namespace ratio