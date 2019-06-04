#include "ov_theory.h"
#include "ov_value_listener.h"
#include <algorithm>
#include <cassert>

namespace smt
{

ov_theory::ov_theory(sat_core &sat) : theory(sat) {}

ov_theory::~ov_theory() {}

var ov_theory::new_var(const std::unordered_set<var_value *> &items, const bool enforce_exct_one)
{
    assert(!items.empty());
    const var id = assigns.size();
    assigns.push_back(std::unordered_map<var_value *, var>());
    if (items.size() == 1)
        assigns.back().emplace(*items.begin(), TRUE_var);
    else
    {
        std::vector<lit> lits;
        lits.reserve(items.size());
        for (const auto &i : items)
        {
            const var bv = sat.new_var();
            assigns.back().emplace(i, bv);
            lits.push_back(bv);
            bind(bv);
            is_contained_in[bv].insert(id);
        }
        if (enforce_exct_one)
        {
            bool exct_one = sat.exct_one(lits);
            assert(exct_one);
        }
    }
    return id;
}

var ov_theory::new_var(const std::vector<var> &vars, const std::vector<var_value *> &vals)
{
    assert(!vars.empty());
    assert(std::all_of(vars.begin(), vars.end(), [&](var v) { return is_contained_in.count(v); }));
    const var id = assigns.size();
    assigns.push_back(std::unordered_map<var_value *, var>());
    for (size_t i = 0; i < vars.size(); ++i)
    {
        assigns.back().emplace(vals.at(i), vars.at(i));
        is_contained_in.at(vars.at(i)).insert(id);
    }
    return id;
}

var ov_theory::allows(const var &v, var_value &val) const
{
    const auto at_right = assigns.at(v).find(&val);
    if (at_right != assigns.at(v).end())
        return at_right->second;
    else
        return FALSE_var;
}

var ov_theory::new_eq(const var &left, const var &right)
{
    if (left == right)
        return TRUE_var;

    if (left > right)
        return new_eq(right, left);

    std::string s_expr = "e" + std::to_string(left) + " == " + "e" + std::to_string(right);
    if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    {
        std::unordered_set<var_value *> intersection;
        for (const auto &v : assigns.at(left))
            if (assigns.at(right).count(v.first))
                intersection.insert(v.first);

        if (intersection.empty())
            return FALSE_var;

        // we need to create a new variable..
        const var e = sat.new_var();
        bool nc;
        for (const auto &v : assigns[left])
            if (!intersection.count(v.first))
            {
                nc = sat.new_clause({lit(e, false), lit(v.second, false)});
                assert(nc);
            }
        for (const auto &v : assigns[right])
            if (!intersection.count(v.first))
            {
                nc = sat.new_clause({lit(e, false), lit(v.second, false)});
                assert(nc);
            }
        for (const auto &v : intersection)
        {
            nc = sat.new_clause({lit(e, false), lit(assigns[left].at(v), false), assigns[right].at(v)});
            assert(nc);
            nc = sat.new_clause({lit(e, false), assigns[left].at(v), lit(assigns[right].at(v), false)});
            assert(nc);
            nc = sat.new_clause({e, lit(assigns[left].at(v), false), lit(assigns[right].at(v), false)});
            assert(nc);
        }
        exprs.emplace(s_expr, e);
        return e;
    }
}

bool ov_theory::eq(const var &left, const var &right, const var &p)
{
    if (left == right)
        return true;

    if (left > right)
        return eq(right, left, p);

    std::string s_expr = "e" + std::to_string(left) + " == " + "e" + std::to_string(right);
    if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return sat.eq(p, at_expr->second);
    else
    {
        std::unordered_set<var_value *> intersection;
        for (const auto &v : assigns.at(left))
            if (assigns.at(right).count(v.first))
                intersection.insert(v.first);

        if (intersection.empty())
            return sat.eq(p, FALSE_var);

        // we need to create a new variable..
        for (const auto &v : assigns[left])
            if (!intersection.count(v.first))
                if (!sat.new_clause({lit(p, false), lit(v.second, false)}))
                    return false;
        for (const auto &v : assigns[right])
            if (!intersection.count(v.first))
                if (!sat.new_clause({lit(p, false), lit(v.second, false)}))
                    return false;
        for (const auto &v : intersection)
        {
            if (!sat.new_clause({lit(p, false), lit(assigns[left].at(v), false), assigns[right].at(v)}))
                return false;
            if (!sat.new_clause({lit(p, false), assigns[left].at(v), lit(assigns[right].at(v), false)}))
                return false;
            if (!sat.new_clause({p, lit(assigns[left].at(v), false), lit(assigns[right].at(v), false)}))
                return false;
        }
        exprs.emplace(s_expr, p);
        return true;
    }
}

std::unordered_set<var_value *> ov_theory::value(var v) const
{
    std::unordered_set<var_value *> vals;
    for (const auto &val : assigns[v])
        if (sat.value(val.second) != False)
            vals.insert(val.first);
    return vals;
}

bool ov_theory::propagate(const lit &p, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    for (const auto &v : is_contained_in.at(p.get_var()))
        if (const auto at_v = listening.find(v); at_v != listening.end())
            for (const auto &l : at_v->second)
                l->ov_value_change(v);
    return true;
}

bool ov_theory::check(std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    return true;
}

void ov_theory::push() { layers.push_back(layer()); }

void ov_theory::pop()
{
    for (const auto &v : layers.back().vars)
        if (const auto at_v = listening.find(v); at_v != listening.end())
            for (const auto &l : at_v->second)
                l->ov_value_change(v);
    layers.pop_back();
}

void ov_theory::listen(const var &v, ov_value_listener *const l) { listening[v].insert(l); }
} // namespace smt