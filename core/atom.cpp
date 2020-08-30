#include "atom.h"
#include "predicate.h"
#include "core.h"
#include "type.h"
#include "field.h"

using namespace smt;

namespace ratio
{

    atom::atom(core &cr, const context ctx, const predicate &pred) : item(cr, ctx, pred), sigma(cr.get_sat_core().new_var()) {}
    atom::~atom() {}

    lit atom::new_eq(item &i) noexcept
    {
        if (this == &i)
            return TRUE_var;
        if (get_type().get_name().compare(i.get_type().get_name()) != 0)
            return FALSE_var;
        else if (var_item *ei = dynamic_cast<var_item *>(&i))
            return ei->new_eq(*this);
        else
        {
            std::vector<lit> eqs;
            std::queue<const type *> q;
            q.push(&get_type());
            while (!q.empty())
            {
                for (const auto &f : q.front()->get_fields())
                    if (!f.second->is_synthetic())
                        eqs.push_back(get(f.first)->new_eq(*i.get(f.first)));
                for (const auto &st : q.front()->get_supertypes())
                    q.push(st);
                q.pop();
            }

            switch (eqs.size())
            {
            case 0:
                return TRUE_var;
            case 1:
                return *eqs.begin();
            default:
                return get_core().get_sat_core().new_conj(eqs);
            }
        }
    }

    bool atom::equates(const item &i) const noexcept
    {
        if (this == &i)
            return true;
        if (get_type().get_name().compare(i.get_type().get_name()) != 0)
            return false;
        else if (const var_item *ei = dynamic_cast<const var_item *>(&i))
            return ei->equates(*this);
        else
        {
            std::queue<const type *> q;
            q.push(&get_type());
            while (!q.empty())
            {
                for (const auto &f : q.front()->get_fields())
                    if (!f.second->is_synthetic())
                        if (!get(f.first)->equates(*i.get(f.first)))
                            return false;
                for (const auto &st : q.front()->get_supertypes())
                    q.push(st);
                q.pop();
            }
            return true;
        }
    }
} // namespace ratio