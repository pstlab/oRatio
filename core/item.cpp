#include "item.h"
#include "core.h"
#include "type.h"
#include "field.h"
#include "riddle_lexer.h"
#include <cassert>

using namespace smt;

namespace ratio
{
    item::item(core &cr, const context ctx, const type &tp) : env(cr, context(ctx)), tp(tp) {}
    item::~item() {}

    lit item::new_eq(item &i) noexcept
    {
        if (this == &i)
            return TRUE_lit;
        else if (var_item *ei = dynamic_cast<var_item *>(&i))
            return ei->new_eq(*this);
        else
            return FALSE_lit;
    }

    bool item::equates(const item &i) const noexcept
    {
        if (this == &i)
            return true;
        else if (const var_item *ei = dynamic_cast<const var_item *>(&i))
            return ei->equates(*this);
        else
            return false;
    }

    json item::to_json() const noexcept
    {
        json j_itm;
        j_itm->set("id", new string_val(std::to_string(reinterpret_cast<uintptr_t>(this))));
        j_itm->set("type", new string_val(tp.get_full_name()));
        if (!env::exprs.empty())
            j_itm->set("exprs", env::to_json());
        return j_itm;
    }

    json item::value_to_json() const noexcept { return new string_val(std::to_string(reinterpret_cast<uintptr_t>(this))); }

    CORE_EXPORT bool_item::bool_item(core &cr, const lit &l) : item(cr, context(&cr), cr.get_type(BOOL_KEYWORD)), l(l) {}
    CORE_EXPORT bool_item::~bool_item() {}

    lit bool_item::new_eq(item &i) noexcept
    {
        if (this == &i)
            return TRUE_lit;
        else if (bool_item *be = dynamic_cast<bool_item *>(&i))
            return get_core().get_sat_core().new_eq(l, be->l);
        else
            return FALSE_lit;
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

    json bool_item::value_to_json() const noexcept
    {
        json j_val;
        j_val->set("lit", new string_val((sign(l) ? "b" : "!b") + std::to_string(variable(l))));
        switch (get_core().get_sat_core().value(l))
        {
        case True:
            j_val->set("val", new string_val("True"));
            break;
        case False:
            j_val->set("val", new string_val("False"));
            break;
        case Undefined:
            j_val->set("val", new string_val("Undefined"));
            break;
        }
        return j_val;
    }

    CORE_EXPORT arith_item::arith_item(core &cr, const type &t, const lin &l) : item(cr, context(&cr), t), l(l) { assert(&t == &cr.get_type(INT_KEYWORD) || &t == &cr.get_type(REAL_KEYWORD) || &t == &cr.get_type(TP_KEYWORD)); }
    CORE_EXPORT arith_item::~arith_item() {}

    lit arith_item::new_eq(item &i) noexcept
    {
        if (this == &i)
            return TRUE_lit;
        else if (arith_item *ae = dynamic_cast<arith_item *>(&i))
            if (get_type().get_name().compare(TP_KEYWORD) == 0 || ae->get_type().get_name().compare(TP_KEYWORD) == 0)
                return get_core().get_rdl_theory().new_eq(l, ae->l);
            else
                return get_core().get_lra_theory().new_eq(l, ae->l);
        else
            return FALSE_lit;
    }

    bool arith_item::equates(const item &i) const noexcept
    {
        if (this == &i)
            return true;
        else if (const arith_item *ae = dynamic_cast<const arith_item *>(&i))
            if (get_type().get_name().compare(TP_KEYWORD) == 0 || ae->get_type().get_name().compare(TP_KEYWORD) == 0)
                return get_core().get_rdl_theory().equates(l, ae->l);
            else
                return get_core().get_lra_theory().equates(l, ae->l);
        else
            return false;
    }

    json arith_item::value_to_json() const noexcept
    {
        const auto [lb, ub] = get_type().get_name().compare(TP_KEYWORD) == 0 ? get_core().get_rdl_theory().bounds(l) : get_core().get_lra_theory().bounds(l);
        const auto val = get_type().get_name().compare(TP_KEYWORD) == 0 ? lb : get_core().get_lra_theory().value(l);

        json j_val;
        j_val->set("lin", new string_val(to_string(l)));
        json j_num_val;
        j_num_val->set("num", new long_val(val.get_rational().numerator()));
        j_num_val->set("den", new long_val(val.get_rational().denominator()));
        if (val.get_infinitesimal() != rational::ZERO)
        {
            json j_inf;
            j_inf->set("num", new long_val(val.get_infinitesimal().numerator()));
            j_inf->set("den", new long_val(val.get_infinitesimal().denominator()));
            j_num_val->set("inf", j_inf);
        }
        if (!is_negative_infinite(lb))
        {
            json j_lb_bound;
            j_lb_bound->set("num", new long_val(lb.get_rational().numerator()));
            j_lb_bound->set("den", new long_val(lb.get_rational().denominator()));
            if (val.get_infinitesimal() != rational::ZERO)
            {
                json j_inf;
                j_inf->set("num", new long_val(lb.get_infinitesimal().numerator()));
                j_inf->set("den", new long_val(lb.get_infinitesimal().denominator()));
                j_lb_bound->set("inf", j_inf);
            }
            j_num_val->set("lb", j_lb_bound);
        }
        if (!is_positive_infinite(ub))
        {
            json j_ub_bound;
            j_ub_bound->set("num", new long_val(ub.get_rational().numerator()));
            j_ub_bound->set("den", new long_val(ub.get_rational().denominator()));
            if (val.get_infinitesimal() != rational::ZERO)
            {
                json j_inf;
                j_inf->set("num", new long_val(ub.get_infinitesimal().numerator()));
                j_inf->set("den", new long_val(ub.get_infinitesimal().denominator()));
                j_ub_bound->set("inf", j_inf);
            }
            j_num_val->set("lb", j_ub_bound);
        }
        j_val->set("val", j_num_val);

        return j_val;
    }

    CORE_EXPORT var_item::var_item(core &cr, const type &t, var ev) : item(cr, context(&cr), t), ev(ev) {}
    CORE_EXPORT var_item::~var_item() {}

    expr var_item::get(const std::string &name) const
    {
        std::map<std::string, const field *> accessible_fields;
        std::queue<const type *> q;
        q.push(&get_type());
        while (!q.empty())
        {
            const auto &flds = q.front()->get_fields();
            accessible_fields.insert(flds.cbegin(), flds.cend());
            for (const auto &st : q.front()->get_supertypes())
                q.push(st);
            q.pop();
        }
        if (!accessible_fields.count(name))
            return item::get(name);
        else
        {
            const auto &it_it = exprs.find(name);
            if (it_it == exprs.cend())
            {
                assert(!get_core().get_ov_theory().value(ev).empty());
                if (auto vs = get_core().get_ov_theory().value(ev); vs.size() == 1)
                    return (static_cast<const item *>(*vs.cbegin()))->get(name);
                else
                {
                    std::unordered_map<item *, std::vector<lit>> val_vars;
                    for (const auto &val : vs)
                        val_vars[&*static_cast<const item *>(val)->get(name)].push_back(get_core().get_ov_theory().allows(ev, *val));
                    if (val_vars.size() == 1)
                        return val_vars.cbegin()->first;

                    std::vector<lit> c_vars;
                    std::vector<item *> c_vals;
                    for (const auto &val : val_vars)
                    {
                        const auto var = get_core().get_sat_core().new_disj(val.second);
                        c_vars.push_back(var);
                        c_vals.push_back(val.first);
                        for (const auto &val_not : val_vars)
                            if (val != val_not)
                                for (const auto &v : val_not.second)
                                {
                                    bool nc = cr.get_sat_core().new_clause({!var, !v});
                                    assert(nc);
                                }
                    }
                    var_expr e = get_core().new_enum(get_type().get_field(name).get_type(), c_vars, c_vals);
                    const_cast<var_item *>(this)->exprs.insert({name, e});
                    return e;
                }
            }
            else
                return it_it->second;
        }
    }

    lit var_item::new_eq(item &i) noexcept
    {
        if (this == &i)
            return TRUE_lit;
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
            std::unordered_set<const var_value *> c_vals = get_core().get_ov_theory().value(ev);
            std::unordered_set<const var_value *> i_vals = get_core().get_ov_theory().value(ei->ev);
            for (const auto &c_v : c_vals)
                if (i_vals.count(c_v))
                    return true;
            return false;
        }
        else
        {
            std::unordered_set<const var_value *> c_vals = get_core().get_ov_theory().value(ev);
            return c_vals.count(const_cast<var_value *>(dynamic_cast<const var_value *>(&i)));
        }
    }

    json var_item::value_to_json() const noexcept
    {
        json j_val;
        j_val->set("var", new string_val("e" + std::to_string(ev)));
        std::vector<json> j_vals;
        std::unordered_set<const var_value *> vals = get_core().get_ov_theory().value(ev);
        for (const auto &val : vals)
            j_vals.push_back(new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const item *>(val)))));
        j_val->set("vals", new array_val(j_vals));
        return j_val;
    }

    CORE_EXPORT string_item::string_item(core &cr, const std::string &l) : item(cr, context(&cr), cr.get_type(STRING_KEYWORD)), l(l) {}
    CORE_EXPORT string_item::~string_item() {}

    lit string_item::new_eq(item &i) noexcept
    {
        if (this == &i)
            return TRUE_lit;
        else if (string_item *se = dynamic_cast<string_item *>(&i))
            return l.compare(se->l) == 0 ? TRUE_lit : FALSE_lit;
        else
            return FALSE_lit;
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

    json string_item::value_to_json() const noexcept { return new string_val(l); }
} // namespace ratio