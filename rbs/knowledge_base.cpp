#include "knowledge_base.h"
#include "wme.h"
#include "predicate.h"
#include "rule.h"

namespace kb
{
    knowledge_base::knowledge_base() {}
    knowledge_base::~knowledge_base()
    {
        // we delete the facts..
        for ([[maybe_unused]] const auto &[f_name, f] : facts)
            delete f;
        // we delete the rules..
        for ([[maybe_unused]] const auto &[r_name, r] : rules)
            delete r;
        // we delete the predicates..
        for ([[maybe_unused]] const auto &[pred_name, pred] : predicates)
            delete pred;
    }
} // namespace kb
