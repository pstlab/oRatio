#pragma once

#include "sat_core.h"
#include <map>

namespace smt
{

class idl_theory : public theory
{
public:
    idl_theory(sat_core &sat, const size_t &size = 16);
    idl_theory(const idl_theory &orig) = delete;
    virtual ~idl_theory();

    var new_var(); // creates and returns a new distance logic variable..

    var new_distance(const var &from, const var &to, const I &dist); // creates and returns a new propositional variable for controlling the constraint 'to - from <= dist'..

    bool distance(const var &from, const var &to, const I &dist, const var &p = TRUE_var); // creates a new constraint 'to - from <= dist' and associates it to the controlling variable 'p'..

    size_t size() const { return _preds.size(); }

private:
    bool propagate(const lit &p, std::vector<lit> &cnfl) override;
    bool check(std::vector<lit> &cnfl) override;
    void push() override;
    void pop() override;

    void propagate(const var &from, const var &to, const I &dist);
    void set_dist(const var &from, const var &to, const I &dist);
    void set_pred(const var &from, const var &to, const var &pred);

    void resize(const size_t &size);

private:
    class idl_distance
    {
        friend class idl_theory;

    public:
        idl_distance(const var &b, const var &from, const var &to, const I &dist) : b(b), from(from), to(to), dist(dist) {}
        idl_distance(const idl_distance &orig) = delete;
        ~idl_distance() {}

    private:
        const var b; // the propositional variable associated to the distance constraint..
        const var from;
        const var to;
        const I dist;
    };

    struct layer
    {
        std::map<std::pair<var, var>, I> old_dists;
        std::map<std::pair<var, var>, var> old_preds;
    };

    size_t n_vars = 0;
    std::vector<std::vector<I>> _dists;
    std::vector<std::vector<var>> _preds;
    std::map<std::pair<var, var>, std::vector<idl_distance *>> dist_constrs;
    std::unordered_map<var, std::vector<idl_distance *>> var_dists;
    std::vector<layer> layers; // we store the updates..
};
} // namespace smt
