#include "ov_theory.h"
#include "ov_value_listener.h"
#include <algorithm>
#include <cassert>

namespace smt
{

    ov_theory::ov_theory(sat_core &sat) : theory(sat) {}
    ov_theory::~ov_theory() {}

    var ov_theory::new_var(const std::vector<var_value *> &items, const bool enforce_exct_one)
    {
        assert(!items.empty());
        const var id = assigns.size();
        assigns.push_back(std::unordered_map<var_value *, lit>());
        if (items.size() == 1)
            assigns.back().emplace(*items.begin(), TRUE_var);
        else
        {
            for (const auto &i : items)
            {
                const var bv = sat.new_var();
                assigns.back().emplace(i, bv);
                bind(bv);
                is_contained_in[bv].insert(id);
            }
            if (enforce_exct_one)
            {
                std::vector<lit> lits;
                lits.reserve(items.size());
                for (const auto &i : items)
                    lits.push_back(assigns.back().find(i)->second);
                bool exct_one = sat.new_clause({sat.new_exct_one(lits)});
                assert(exct_one);
            }
        }
        return id;
    }

    var ov_theory::new_var(const std::vector<lit> &lits, const std::vector<var_value *> &vals)
    {
        assert(!lits.empty());
        assert(std::all_of(lits.begin(), lits.end(), [this](lit p) { return is_contained_in.count(variable(p)); }));
        const var id = assigns.size();
        assigns.push_back(std::unordered_map<var_value *, lit>());
        for (size_t i = 0; i < lits.size(); ++i)
        {
            assigns.back().emplace(vals[i], lits[i]);
            is_contained_in.at(variable(lits[i])).insert(id);
        }
        return id;
    }

    lit ov_theory::allows(const var &v, var_value &val) const
    {
        if (const auto at_right = assigns[v].find(&val); at_right != assigns[v].end())
            return at_right->second;
        else
            return FALSE_var;
    }

    lit ov_theory::new_eq(const var &left, const var &right)
    {
        if (left == right)
            return TRUE_var;

        if (left > right)
            return new_eq(right, left);

        const std::string s_expr = left < right ? ("=e" + std::to_string(left) + "e" + std::to_string(right)) : ("=e" + std::to_string(right) + "e" + std::to_string(left));
        if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
            return at_expr->second;
        else
        {
            std::unordered_set<var_value *> intersection;
            for (const auto &v : assigns[left])
                if (assigns[right].count(v.first))
                    intersection.insert(v.first);

            if (intersection.empty())
                return FALSE_var;

            // we need to create a new variable..
            const var eq_var = sat.new_var(); // the equality variable..

            bool nc;
            // the values outside the intersection are pruned if the equality control variable becomes true..
            for (const auto &v : assigns[left])
                if (!intersection.count(v.first))
                {
                    nc = sat.new_clause({lit(eq_var, false), !v.second});
                    assert(nc);
                }
            for (const auto &v : assigns[right])
                if (!intersection.count(v.first))
                {
                    nc = sat.new_clause({lit(eq_var, false), !v.second});
                    assert(nc);
                }
            // the values inside the intersection are made pairwise equal if the equality variable becomes true..
            for (const auto &v : intersection)
            {
                nc = sat.new_clause({lit(eq_var, false), !assigns[left].at(v), assigns[right].at(v)});
                assert(nc);
                nc = sat.new_clause({lit(eq_var, false), assigns[left].at(v), !assigns[right].at(v)});
                assert(nc);
                nc = sat.new_clause({eq_var, !assigns[left].at(v), !assigns[right].at(v)});
                assert(nc);
            }

            exprs.emplace(s_expr, eq_var);
            return eq_var;
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

    bool ov_theory::propagate(const lit &p)
    {
        assert(cnfl.empty());
        for (const auto &v : is_contained_in.at(variable(p)))
            if (const auto at_v = listening.find(v); at_v != listening.end())
                for (const auto &l : at_v->second)
                    l->ov_value_change(v);
        return true;
    }

    bool ov_theory::check()
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