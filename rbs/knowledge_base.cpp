#include "knowledge_base.h"
#include "fact.h"
#include "node.h"
#include "rule.h"
#include <cassert>

namespace kb
{
    knowledge_base::knowledge_base() {}
    knowledge_base::~knowledge_base()
    {
        // we delete the facts..
        for ([[maybe_unused]] const auto &[f_name, f] : facts)
            delete f;
        // we delete the predicates..
        for ([[maybe_unused]] const auto &[pred_name, pred] : predicates)
            delete pred;
        // we delete the rules..
        for ([[maybe_unused]] const auto &[r_name, r] : rules)
            delete r;
    }

    predicate &knowledge_base::create_predicate(const std::string &p_name)
    {
        assert(!exists_predicate(p_name));
        predicate *p = new predicate(p_name);
        predicates[p_name] = p;
        return *p;
    }
} // namespace kb
