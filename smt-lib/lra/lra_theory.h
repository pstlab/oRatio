#pragma once

#include "theory.h"
#include "lin.h"
#include "inf_rational.h"
#include <set>
#include <unordered_map>

namespace smt
{

class lra_value_listener;
class assertion;
class row;

class lra_theory : public theory
{
    friend class lra_value_listener;
    friend class assertion;
    friend class row;

  public:
    lra_theory(sat_core &sat);
    lra_theory(const lra_theory &orig) = delete;
    virtual ~lra_theory();

    const var new_var();
    const var new_var(const lin &l);

    const var new_lt(const lin &left, const lin &right);
    const var new_leq(const lin &left, const lin &right);
    const var new_geq(const lin &left, const lin &right);
    const var new_gt(const lin &left, const lin &right);

    inf_rational lb(const var &v) const { return assigns.at(lb_index(v)).value; } // the current lower bound of variable 'v'..
    inf_rational ub(const var &v) const { return assigns.at(ub_index(v)).value; } // the current upper bound of variable 'v'..
    inf_rational value(const var &v) const { return vals.at(v); }                 // the current value of variable 'v'..

    inf_rational lb(const lin &l) const // the current lower bound of linear expression 'l'..
    {
        inf_rational b(l.known_term);
        for (const auto &term : l.vars)
            b += (term.second.is_positive() ? lb(term.first) : ub(term.first)) * term.second;
        return b;
    }
    inf_rational ub(const lin &l) const // the current upper bound of linear expression 'l'..
    {
        inf_rational b(l.known_term);
        for (const auto &term : l.vars)
            b += (term.second.is_positive() ? ub(term.first) : lb(term.first)) * term.second;
        return b;
    }
    inf_rational value(const lin &l) const
    {
        inf_rational v(l.known_term);
        for (const auto &term : l.vars)
            v += value(term.first) * term.second;
        return v;
    }

  private:
    bool propagate(const lit &p, std::vector<lit> &cnfl) override;
    bool check(std::vector<lit> &cnfl) override;
    void push() override;
    void pop() override;

    bool assert_lower(const var &x_i, const inf_rational &val, const lit &p, std::vector<lit> &cnfl);
    bool assert_upper(const var &x_i, const inf_rational &val, const lit &p, std::vector<lit> &cnfl);
    void update(const var &x_i, const inf_rational &v);
    void pivot_and_update(const var &x_i, const var &x_j, const inf_rational &v);
    void pivot(const var x_i, const var x_j);

    void listen(const var &v, lra_value_listener *const l) { listening[v].insert(l); }

    static size_t lb_index(const var &v) { return v << 1; }       // the index of the lower bound of the 'v' variable..
    static size_t ub_index(const var &v) { return (v << 1) ^ 1; } // the index of the upper bound of the 'v' variable..

  public:
    std::string to_string();

  private:
    /**
     * Represents the bound of a variable and the reason for its existence.
     */
    struct bound
    {
        inf_rational value; // the value of the bound..
        lit *reason;        // the reason for the value..
    };

    std::vector<bound> assigns;                            // the current assignments..
    std::vector<inf_rational> vals;                        // the current values..
    std::map<var, row *> tableau;                          // the sparse matrix..
    std::unordered_map<std::string, var> exprs;            // the expressions (string to numeric variable) for which already exist slack variables..
    std::unordered_map<std::string, var> s_asrts;          // the assertions (string to boolean variable) used for reducing the number of boolean variables..
    std::unordered_map<var, assertion *> v_asrts;          // the assertions (boolean variable to assertion) used for enforcing (negating) assertions..
    std::vector<std::vector<assertion *>> a_watches;       // for each variable 'v', a list of assertions watching 'v'..
    std::vector<std::set<row *>> t_watches;                // for each variable 'v', a list of tableau rows watching 'v'..
    std::vector<std::unordered_map<size_t, bound>> layers; // we store the updated bounds..
    std::unordered_map<var, std::set<lra_value_listener *>> listening;
};
}