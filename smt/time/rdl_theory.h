#pragma once

#include "sat_core.h"
#include "inf_rational.h"
#include <map>

namespace smt
{

class rdl_theory : public theory
{
public:
    rdl_theory(sat_core &sat, const size_t &size = 16);
    rdl_theory(const rdl_theory &orig) = delete;
    virtual ~rdl_theory();

    var new_var(); // creates and returns a new distance logic variable..

    var new_distance(const var &from, const var &to, const inf_rational &dist); // creates and returns a new propositional variable for controlling the constraint 'to - from <= dist'..

    bool distance(const var &from, const var &to, const inf_rational &dist, const var &p = TRUE_var); // creates a new constraint 'to - from <= dist' and associates it to the controlling variable 'p'..

    size_t size() const { return _preds.size(); }

private:
    bool propagate(const lit &p, std::vector<lit> &cnfl) override;
    bool check(std::vector<lit> &cnfl) override;
    void push() override;
    void pop() override;

    void propagate(const var &from, const var &to, const inf_rational &dist);
    void set_dist(const var &from, const var &to, const inf_rational &dist);
    void set_pred(const var &from, const var &to, const var &pred);

    void resize(const size_t &size);

public:
    static constexpr var origin() { return 0; }
    static constexpr var horizon() { return 1; }

private:
    class rdl_distance
    {
        friend class rdl_theory;

    public:
        rdl_distance(const var &b, const var &from, const var &to, const inf_rational &dist) : b(b), from(from), to(to), dist(dist) {}
        rdl_distance(const rdl_distance &orig) = delete;
        ~rdl_distance() {}

    private:
        const var b; // the propositional variable associated to the distance constraint..
        const var from;
        const var to;
        const inf_rational dist;
    };

    struct layer
    {
        std::map<std::pair<var, var>, inf_rational> old_dists;
        std::map<std::pair<var, var>, var> old_preds;
    };

    size_t n_vars = 0;
    std::vector<std::vector<inf_rational>> _dists;
    std::vector<std::vector<var>> _preds;
    std::map<std::pair<var, var>, std::vector<rdl_distance *>> dist_constrs;
    std::unordered_map<var, std::vector<rdl_distance *>> var_dists;
    std::vector<layer> layers; // we store the updates..
};
} // namespace smt
