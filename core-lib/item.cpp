#include "item.h"
#include "core.h"
#include "type.h"
#include "field.h"
#include "ov_theory.h"
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
    if (tp.name.compare(i.tp.name) != 0)
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
                if (!f.second->synthetic)
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
            return eqs.begin()->v;
        default:
            return cr.sat_cr.new_conj(eqs);
        }
    }
}

bool item::equates(const item &i) const noexcept
{
    if (this == &i)
        return true;
    if (tp.name.compare(i.tp.name) != 0)
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
                if (!f.second->synthetic)
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
        return cr.sat_cr.new_eq(l, be->l);
    else
        return FALSE_var;
}

bool bool_item::equates(const item &i) const noexcept
{
    if (this == &i)
        return true;
    else if (const bool_item *be = dynamic_cast<const bool_item *>(&i))
    {
        lbool c_val = cr.sat_cr.value(l);
        lbool i_val = cr.sat_cr.value(be->l);
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
        return cr.sat_cr.new_conj({cr.lra_th.new_leq(l, ae->l), cr.lra_th.new_geq(l, ae->l)});
    else
        return FALSE_var;
}

bool arith_item::equates(const item &i) const noexcept
{
    if (this == &i)
        return true;
    else if (const arith_item *ae = dynamic_cast<const arith_item *>(&i))
        return cr.lra_th.ub(l) >= cr.lra_th.lb(ae->l) && cr.lra_th.lb(l) <= cr.lra_th.ub(ae->l); // the two intervals intersect..
    else
        return false;
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

var_item::var_item(core &cr, const type &t, var ev) : item(cr, &cr, t), ev(ev) {}
var_item::~var_item() {}

expr var_item::get(const std::string &name) const
{
    std::map<std::string, field *> accessible_fields;
    std::queue<const type *> q;
    q.push(&tp);
    while (!q.empty())
    {
        for (const auto &f : q.front()->get_fields())
            accessible_fields.insert(f);
        for (const auto &st : q.front()->get_supertypes())
            q.push(st);
        q.pop();
    }
    const auto &f_it = accessible_fields.find(name);
    if (f_it == accessible_fields.end())
        return item::get(name);
    else
    {
        const auto &it_it = items.find(name);
        if (it_it == items.end())
        {
            std::unordered_set<var_value *> vs = cr.ov_th.value(ev);
            assert(!vs.empty());
            if (vs.size() == 1)
                return (static_cast<item *>(*vs.begin()))->get(name);
            else
            {
                std::vector<var> c_vars;
                std::vector<item *> c_vals;
                std::unordered_set<item *> vals_set;
                for (const auto &val : vs)
                {
                    c_vars.push_back(cr.ov_th.allows(ev, *val));
                    c_vals.push_back(&*static_cast<item *>(val)->get(name));
                    vals_set.insert(c_vals.back());
                }
                if (vals_set.size() == 1)
                    return *vals_set.begin();
                var_expr e = cr.new_enum(tp.get_field(name).tp, c_vars, c_vals);
                const_cast<var_item *>(this)->items.insert({name, e});
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
        return cr.ov_th.eq(ev, ee->ev);
    else
        return cr.ov_th.allows(ev, i);
}

bool var_item::equates(const item &i) const noexcept
{
    if (this == &i)
        return true;
    else if (const var_item *ei = dynamic_cast<const var_item *>(&i))
    {
        std::unordered_set<var_value *> c_vals = cr.ov_th.value(ev);
        std::unordered_set<var_value *> i_vals = cr.ov_th.value(ei->ev);
        for (const auto &c_v : c_vals)
            if (i_vals.find(c_v) != i_vals.end())
                return true;
        return false;
    }
    else
    {
        std::unordered_set<var_value *> c_vals = cr.ov_th.value(ev);
        return c_vals.find(const_cast<var_value *>(static_cast<const var_value *>(&i))) != c_vals.end();
    }
}
}