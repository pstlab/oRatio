#include "atom.h"
#include "predicate.h"
#include "core.h"
#include "type.h"
#include "field.h"

using namespace smt;

namespace ratio
{
    atom::atom(core &cr, const context ctx, const predicate &pred) : item(cr, context(ctx), pred), sigma(cr.get_sat_core().new_var()) {}
    atom::~atom() {}

    lit atom::new_eq(item &i) noexcept
    {
        if (this == &i)
            return TRUE_lit;
        if (get_type().get_name().compare(i.get_type().get_name()) != 0)
            return FALSE_lit;
        else if (var_item *ei = dynamic_cast<var_item *>(&i))
            return ei->new_eq(*this);
        else
        {
            std::vector<lit> eqs;
            std::queue<const type *> q;
            q.push(&get_type());
            while (!q.empty())
            {
                for (const auto &[f_name, f] : q.front()->get_fields())
                    if (!f->is_synthetic())
                        eqs.push_back(get(f_name)->new_eq(*i.get(f_name)));
                for (const auto &st : q.front()->get_supertypes())
                    q.push(st);
                q.pop();
            }

            switch (eqs.size())
            {
            case 0:
                return TRUE_lit;
            case 1:
                return *eqs.cbegin();
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
                for (const auto &[f_name, f] : q.front()->get_fields())
                    if (!f->is_synthetic())
                        if (!get(f_name)->equates(*i.get(f_name)))
                            return false;
                for (const auto &st : q.front()->get_supertypes())
                    q.push(st);
                q.pop();
            }
            return true;
        }
    }

    json atom::to_json() const noexcept
    {
        json j_atm;
        j_atm->set("id", new string_val(std::to_string(reinterpret_cast<uintptr_t>(this))));
        j_atm->set("predicate", new string_val(get_type().get_full_name()));
        j_atm->set("sigma", new string_val(std::to_string(sigma)));
        switch (get_core().get_sat_core().value(sigma))
        {
        case True:
            j_atm->set("state", new string_val("Active"));
            break;
        case False:
            j_atm->set("state", new string_val("Unified"));
            break;
        case Undefined:
            j_atm->set("state", new string_val("Inactive"));
            break;
        }
        j_atm->set("pars", env::to_json());
        return j_atm;
    }
} // namespace ratio