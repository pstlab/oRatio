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

    var new_var(); // creates and returns a new difference logic variable..

    var new_difference(const var &from, const var &to, const inf_rational &diff);

    bool difference(const var &from, const var &to, const inf_rational &diff, const var &p = TRUE_var);

    size_t size() const { return _preds.size(); }

private:
    bool propagate(const lit &p, std::vector<lit> &cnfl) override;
    bool check(std::vector<lit> &cnfl) override;
    void push() override;
    void pop() override;

    void propagate(const size_t &from, const size_t &to, const inf_rational &diff);
    void set_diff(const size_t &from, const size_t &to, const inf_rational &diff);

    void resize(const size_t &size);

public:
    static constexpr size_t origin() { return 0; }
    static constexpr size_t horizon() { return 1; }

private:
    class rdl_difference
    {
        friend class rdl_theory;

    public:
        rdl_difference(const var &b, const var &from, const var &to, const inf_rational &diff) : b(b), from(from), to(to), diff(diff) {}
        rdl_difference(const rdl_difference &orig) = delete;
        ~rdl_difference() {}

    private:
        const var b; // the propositional variable associated to the difference constraint..
        const var from;
        const var to;
        const inf_rational diff;
    };

    struct layer
    {
        std::map<std::pair<var, var>, inf_rational> old_dists;
        std::unordered_map<var, var> old_preds;
    };

    std::vector<std::vector<inf_rational>> _data;
    std::vector<var> _preds;
    std::map<std::pair<var, var>, std::vector<rdl_difference *>> diff_constrs;
    std::unordered_map<var, std::vector<rdl_difference *>> var_diffs;
#ifndef STRONG_PROPAGATION
    std::queue<rdl_difference *> prop_q;
#endif
    std::vector<layer> layers; // we store the updates..
};
} // namespace smt
