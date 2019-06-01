#include "item.h"
#include "core.h"
#include "type.h"
#include "field.h"
#include <cassert>

using namespace smt;

namespace ratio
{

item::item(core &cr, const context ctx, const type &tp) : env(cr, ctx), tp(tp) {}
item::~item() {}

var item::eq(item &i) noexcept
{
    if (this == &i)
        return TRUE_var;
    if (tp.get_name().compare(i.tp.get_name()) != 0)
        return FALSE_var;
    else if (var_item *ei = dynamic_cast<var_item *>(&i))
        return ei->eq(*this);
    else
    {
        std::vector<lit> eqs;
        std::queue<const type *> q;
        q.push(&tp);
        while (!q.empty())
        {
            for (const auto &f : q.front()->get_fields())
                if (!f.second->is_synthetic())
                    eqs.push_back(get(f.first)->eq(*i.get(f.first)));
            for (const auto &st : q.front()->get_supertypes())
                q.push(st);
            q.pop();
        }

        switch (eqs.size())
        {
        case 0:
            return TRUE_var;
        case 1:
            return eqs.begin()->get_var();
        default:
            return get_core().get_sat_core().new_conj(eqs);
        }
    }
}

bool item::equates(const item &i) const noexcept
{
    if (this == &i)
        return true;
    if (tp.get_name().compare(i.tp.get_name()) != 0)
        return false;
    else if (const var_item *ei = dynamic_cast<const var_item *>(&i))
        return ei->equates(*this);
    else
    {
        std::queue<const type *> q;
        q.push(&tp);
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

bool_item::bool_item(core &cr, const lit &l) : item(cr, &cr, cr.get_type(BOOL_KEYWORD)), l(l) {}
bool_item::~bool_item() {}

var bool_item::eq(item &i) noexcept
{
    if (this == &i)
        return TRUE_var;
    else if (bool_item *be = dynamic_cast<bool_item *>(&i))
        return get_core().get_sat_core().new_eq(l, be->l);
    else
        return FALSE_var;
}

bool bool_item::equates(const item &i) const noexcept
{
    if (this == &i)
        return true;
    else if (const bool_item *be = dynamic_cast<const bool_item *>(&i))
    {
        lbool c_val = get_core().get_sat_core().value(l);
        lbool i_val = get_core().get_sat_core().value(be->l);
        return c_val == i_val || c_val == Undefined || i_val == Undefined;
    }
    else
        return false;
}

arith_item::arith_item(core &cr, const type &t, const lin &l) : item(cr, &cr, t), l(l) { assert(&t == &cr.get_type(INT_KEYWORD) || &t == &cr.get_type(REAL_KEYWORD)); }
arith_item::~arith_item() {}

var arith_item::eq(item &i) noexcept
{
    if (this == &i)
        return TRUE_var;
    else if (arith_item *ae = dynamic_cast<arith_item *>(&i))
        return get_core().get_sat_core().new_conj({get_core().get_lra_theory().new_leq(l, ae->l), get_core().get_lra_theory().new_geq(l, ae->l)});
    else
        return FALSE_var;
}

bool arith_item::equates(const item &i) const noexcept
{
    if (this == &i)
        return true;
    else if (const arith_item *ae = dynamic_cast<const arith_item *>(&i))
        return get_core().get_lra_theory().ub(l) >= get_core().get_lra_theory().lb(ae->l) && get_core().get_lra_theory().lb(l) <= get_core().get_lra_theory().ub(ae->l); // the two intervals intersect..
    else
        return false;
}

var_item::var_item(core &cr, const type &t, var ev) : item(cr, &cr, t), ev(ev) {}
var_item::~var_item() {}

expr var_item::get(const std::string &name) const
{
    std::map<std::string, const field *> accessible_fields;
    std::queue<const type *> q;
    q.push(&get_type());
    while (!q.empty())
    {
        const auto flds = q.front()->get_fields();
        accessible_fields.insert(flds.cbegin(), flds.cend());
        for (const auto &st : q.front()->get_supertypes())
            q.push(st);
        q.pop();
    }
    if (const auto &f_it = accessible_fields.find(name); f_it == accessible_fields.end())
        return item::get(name);
    else
    {
        const auto &it_it = exprs.find(name);
        if (it_it == exprs.end())
        {
            assert(!get_core().get_ov_theory().value(ev).empty());
            if (std::unordered_set<var_value *> vs = get_core().get_ov_theory().value(ev); vs.size() == 1)
                return (static_cast<item *>(*vs.begin()))->get(name);
            else
            {
                std::vector<var> c_vars;
                std::vector<item *> c_vals;
                std::unordered_set<item *> vals_set;
                for (const auto &val : vs)
                {
                    c_vars.push_back(get_core().get_ov_theory().allows(ev, *val));
                    c_vals.push_back(&*static_cast<item *>(val)->get(name));
                    vals_set.insert(c_vals.back());
                }
                if (vals_set.size() == 1)
                    return *vals_set.begin();
                var_expr e = get_core().new_enum(get_type().get_field(name).get_type(), c_vars, c_vals);
                const_cast<var_item *>(this)->exprs.insert({name, e});
                return e;
            }
        }
        else
            return it_it->second;
    }
}

var var_item::eq(item &i) noexcept
{
    if (this == &i)
        return TRUE_var;
    else if (var_item *ee = dynamic_cast<var_item *>(&i))
        return get_core().get_ov_theory().new_eq(ev, ee->ev);
    else
        return get_core().get_ov_theory().allows(ev, i);
}

bool var_item::equates(const item &i) const noexcept
{
    if (this == &i)
        return true;
    else if (const var_item *ei = dynamic_cast<const var_item *>(&i))
    {
        std::unordered_set<var_value *> c_vals = get_core().get_ov_theory().value(ev);
        std::unordered_set<var_value *> i_vals = get_core().get_ov_theory().value(ei->ev);
        for (const auto &c_v : c_vals)
            if (i_vals.find(c_v) != i_vals.end())
                return true;
        return false;
    }
    else
    {
        std::unordered_set<var_value *> c_vals = get_core().get_ov_theory().value(ev);
        return c_vals.find(const_cast<var_value *>(dynamic_cast<const var_value *>(&i))) != c_vals.end();
    }
}

string_item::string_item(core &cr, const std::string &l) : item(cr, &cr, cr.get_type(STRING_KEYWORD)), l(l) {}
string_item::~string_item() {}

var string_item::eq(item &i) noexcept
{
    if (this == &i)
        return TRUE_var;
    else if (string_item *se = dynamic_cast<string_item *>(&i))
        return l.compare(se->l) == 0 ? TRUE_var : FALSE_var;
    else
        return FALSE_var;
}

bool string_item::equates(const item &i) const noexcept
{
    if (this == &i)
        return true;
    else if (const string_item *se = dynamic_cast<const string_item *>(&i))
        return l.compare(se->l) == 0;
    else
        return false;
}
} // namespace ratio
