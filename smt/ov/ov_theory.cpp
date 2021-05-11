#include "ov_theory.h"
#include "ov_value_listener.h"
#include <algorithm>
#include <cassert>

namespace smt
{
    SMT_EXPORT ov_theory::ov_theory(sat_core &sat) : theory(sat) {}
    SMT_EXPORT ov_theory::~ov_theory() {}

    SMT_EXPORT var ov_theory::new_var(const std::vector<var_value *> &items, const bool enforce_exct_one) noexcept
    {
        assert(!items.empty());
        const var id = assigns.size();
        auto c_vals = std::unordered_map<const var_value *, lit>();
        if (items.size() == 1)
            c_vals.emplace(*items.cbegin(), TRUE_lit);
        else
        {
            for (const auto &i : items)
            {
                const var bv = sat.new_var();
                c_vals.emplace(i, lit(bv));
                bind(bv);
                is_contained_in[bv].insert(id);
            }
            if (enforce_exct_one)
            {
                std::vector<lit> lits;
                lits.reserve(items.size());
                for (const auto &i : items)
                    lits.push_back(c_vals.find(i)->second);
                bool exct_one = sat.new_clause({sat.new_exct_one(lits)});
                assert(exct_one);
            }
        }
        assigns.push_back(c_vals);
        return id;
    }

    SMT_EXPORT var ov_theory::new_var(const std::vector<lit> &lits, const std::vector<var_value *> &vals) noexcept
    {
        assert(!lits.empty());
        assert(lits.size() == vals.size());
        const var id = assigns.size();
        auto c_vals = std::unordered_map<const var_value *, lit>();
        for (size_t i = 0; i < lits.size(); ++i)
        {
            c_vals.emplace(vals[i], lits[i]);
            is_contained_in[variable(lits[i])].insert(id);
        }
        assigns.push_back(c_vals);
        return id;
    }

    SMT_EXPORT lit ov_theory::allows(const var &v, const var_value &val) const noexcept
    {
        if (const auto at_right = assigns[v].find(&val); at_right != assigns[v].cend())
            return at_right->second;
        else
            return FALSE_lit;
    }

    SMT_EXPORT lit ov_theory::new_eq(const var &left, const var &right) noexcept
    {
        if (left == right)
            return TRUE_lit;

        if (left > right)
            return new_eq(right, left);

        const std::string s_expr = left < right ? ("=e" + std::to_string(left) + "e" + std::to_string(right)) : ("=e" + std::to_string(right) + "e" + std::to_string(left));
        if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.cend()) // the expression already exists..
            return at_expr->second;
        else
        {
            std::unordered_set<const var_value *> intersection;
            for (const auto &[val, l] : assigns[left])
                if (assigns[right].count(val))
                    intersection.insert(val);

            if (intersection.empty())
                return FALSE_lit;

            // we need to create a new variable..
            const lit eq_lit = lit(sat.new_var()); // the equality literal..

            bool nc;
            // the values outside the intersection are pruned if the equality control variable becomes true..
            for (const auto &[val, l] : assigns[left])
                if (!intersection.count(val))
                {
                    nc = sat.new_clause({!eq_lit, !l});
                    assert(nc);
                }
            for (const auto &[val, l] : assigns[right])
                if (!intersection.count(val))
                {
                    nc = sat.new_clause({!eq_lit, !l});
                    assert(nc);
                }
            // the values inside the intersection are made pairwise equal if the equality variable becomes true..
            for (const auto &v : intersection)
            {
                nc = sat.new_clause({!eq_lit, !assigns[left].at(v), assigns[right].at(v)});
                assert(nc);
                nc = sat.new_clause({!eq_lit, assigns[left].at(v), !assigns[right].at(v)});
                assert(nc);
                nc = sat.new_clause({eq_lit, !assigns[left].at(v), !assigns[right].at(v)});
                assert(nc);
            }

            exprs.emplace(s_expr, eq_lit);
            return eq_lit;
        }
    }

    SMT_EXPORT std::unordered_set<const var_value *> ov_theory::value(var v) const noexcept
    {
        std::unordered_set<const var_value *> vals;
        for (const auto &[val, l] : assigns[v])
            if (sat.value(l) != False)
                vals.insert(val);
        return vals;
    }

    bool ov_theory::propagate(const lit &p) noexcept
    { // propagation is performed at SAT level, here we just notify possible listeners..
        assert(cnfl.empty());
        for (const auto &v : is_contained_in.at(variable(p)))
            if (const auto at_v = listening.find(v); at_v != listening.cend())
                for (const auto &l : at_v->second)
                    l->ov_value_change(v);
        return true;
    }

    bool ov_theory::check() noexcept
    {
        assert(cnfl.empty());
        return true;
    }

    void ov_theory::push() noexcept { layers.push_back(layer()); }

    void ov_theory::pop() noexcept
    {
        for (const auto &v : layers.back().vars)
            if (const auto at_v = listening.find(v); at_v != listening.cend())
                for (const auto &l : at_v->second)
                    l->ov_value_change(v);
        layers.pop_back();
    }
} // namespace smt