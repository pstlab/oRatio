#include "rdl_theory.h"
#include <cassert>

namespace smt
{

rdl_theory::rdl_theory(sat_core &sat, const size_t &size) : theory(sat), _data(std::vector<std::vector<inf_rational>>(size, std::vector<inf_rational>(size, rational::POSITIVE_INFINITY)))
{
    for (size_t i = 0; i < size; ++i)
        _data[i][i] = 0;
}

rdl_theory::~rdl_theory() {}

var rdl_theory::new_var()
{
    var tp = _preds.size();
    if (_data.size() == tp)
        resize((_data.size() * 3) / 2 + 1);
    _preds.push_back(tp);
    return tp;
}

var rdl_theory::new_difference(const var &from, const var &to, const inf_rational &diff)
{
    if (_data[to][from] < -diff)
        return FALSE_var; // the constraint is inconsistent..
    else if (_data[from][to] <= diff)
        return TRUE_var; // the constraint is redundant..
    else
    { // we need to create a new propositional variable..
        const var ctr = sat.new_var();
        bind(ctr);
        rdl_difference *c_diff = new rdl_difference(ctr, from, to, diff);
        var_diffs[ctr].push_back(c_diff);
        diff_constrs[{from, to}].push_back(c_diff);
        return ctr;
    }
}

bool rdl_theory::difference(const var &from, const var &to, const inf_rational &diff, const var &p)
{
    if (_data[to][from] < -diff)
        return sat.eq(p, FALSE_var); // the constraint is inconsistent..
    else if (_data[from][to] <= diff)
        return sat.eq(p, TRUE_var); // the constraint is redundant..
    else
        switch (sat.value(p))
        {
        case True:
        {
            propagate(from, to, diff);
            return true;
        }
        case False:
        {
            propagate(to, from, -diff - inf_rational(rational::ZERO, rational::ONE));
            return true;
        }
        default:
            bind(p);
            rdl_difference *c_diff = new rdl_difference(p, from, to, diff);
            var_diffs[p].push_back(c_diff);
            diff_constrs[{from, to}].push_back(c_diff);
            return true;
        }
}

bool rdl_theory::propagate(const lit &p, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    assert(var_diffs.count(p.get_var()));
    if (p.get_sign())
    { // the assertion is direct..
        for (const auto &diff : var_diffs.at(p.get_var()))
        {
            if (_data[diff->to][diff->from] < -diff->diff)
            { // we build the cause for the conflict..
                var c_from = diff->from;
                while (c_from != diff->to)
                {
                    if (const auto &c_d = diff_constrs.find({_preds[c_from], c_from}); c_d != diff_constrs.end())
                        for (const auto &c_diff : c_d->second)
                            cnfl.push_back(lit(c_diff->b, false));
                    c_from = _preds[c_from];
                }
                cnfl.push_back(!p);
                return false;
            }

            if (_data[diff->from][diff->to] > diff->diff)
                propagate(diff->from, diff->to, diff->diff);
        }
    }
    else
    { // the assertion is negated..
        for (const auto &diff : var_diffs.at(p.get_var()))
        {
            if (_data[diff->from][diff->to] <= diff->diff)
            { // we build the cause for the conflict..
                var c_from = diff->from;
                while (c_from != diff->to)
                {
                    if (const auto &c_d = diff_constrs.find({_preds[c_from], c_from}); c_d != diff_constrs.end())
                        for (const auto &c_diff : c_d->second)
                            cnfl.push_back(lit(c_diff->b, false));
                    c_from = _preds[c_from];
                }
                cnfl.push_back(!p);
                return false;
            }

            if (_data[diff->to][diff->from] >= -diff->diff)
                propagate(diff->to, diff->from, -diff->diff - inf_rational(rational::ZERO, rational::ONE));
        }
    }
    return true;
}

bool rdl_theory::check(std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    return true;
}

void rdl_theory::push() { layers.push_back(layer()); }

void rdl_theory::pop()
{
    for (const auto &dist : layers.back().old_dists)
        _data[dist.first.first][dist.first.second] = dist.second;
    for (const auto &pred : layers.back().old_preds)
        _preds[pred.first] = pred.second;
    layers.pop_back();
}

void rdl_theory::propagate(const size_t &from, const size_t &to, const inf_rational &diff)
{
    set_diff(from, to, diff);
    std::vector<size_t> set_i;
    std::vector<size_t> set_j;
    std::vector<std::pair<var, var>> c_updates;

    // start with an O(n) loop
    for (size_t u = 0; u < size(); ++u)
    {
        if (_data[u][from] < _data[u][to] - diff)
        { // u -> from -> to is shorter than u -> to
            set_diff(u, to, _data[u][from] + diff);
            set_i.push_back(u);
            c_updates.push_back({u, to});
        }
        if (_data[to][u] < _data[from][u] - diff)
        { // from -> to -> u is shorter than from -> u
            set_diff(from, u, _data[to][u] + diff);
            set_j.push_back(u);
            c_updates.push_back({from, u});
        }
    }

    // finally, loop over set_i and set_j in O(n^2) time (but possibly much less)
    for (const auto &i : set_i)
        for (const auto &j : set_j)
            if (i != j && _data[i][to] + _data[to][j] < _data[i][j])
            { // i -> from -> to -> j is shorter than i -> j
                set_diff(i, j, _data[i][to] + _data[to][j]);
                c_updates.push_back({i, j});
            }

#ifdef STRONG_PROPAGATION
    for (const auto &c_pairs : c_updates)
        for (const auto &diff : diff_constrs[c_pairs])
            if (sat.value(diff->b) == Undefined)
                if (_data[diff->to][diff->from] < -diff->diff)
                { // the constraint is inconsistent..
                    std::vector<lit> cnfl;
                    cnfl.push_back(lit(diff->b, false));
                    var c_from = diff->from;
                    while (c_from != diff->to)
                    {
                        if (const auto &c_d = diff_constrs.find({_preds[c_from], c_from}); c_d != diff_constrs.end())
                            for (const auto &c_diff : c_d->second)
                                if (sat.value(c_diff->b) != Undefined)
                                    cnfl.push_back(lit(c_diff->b, false));
                        c_from = _preds[c_from];
                    }
                    // we propagate the reason for assigning false to diff->b..
                    record(cnfl);
                }
                else if (_data[diff->from][diff->to] <= diff->diff)
                { // the constraint is redundant..
                    std::vector<lit> cnfl;
                    cnfl.push_back(diff->b);
                    var c_from = diff->from;
                    while (c_from != diff->to)
                    {
                        if (const auto &c_d = diff_constrs.find({_preds[c_from], c_from}); c_d != diff_constrs.end())
                            for (const auto &c_diff : c_d->second)
                                if (sat.value(c_diff->b) != Undefined)
                                    cnfl.push_back(lit(c_diff->b, false));
                        c_from = _preds[c_from];
                    }
                    // we propagate the reason for assigning true to diff->b..
                    record(cnfl);
                }
#endif
}

void rdl_theory::set_diff(const size_t &from, const size_t &to, const inf_rational &diff)
{
    assert(_data[from][to] > diff);
    if (!layers.empty())
    { // we store the current values for backtracking purposes..
        if (!layers.back().old_dists.count({from, to}))
            layers.back().old_dists.insert({{from, to}, _data[from][to]});
        if (!layers.back().old_preds.count(to))
            layers.back().old_preds.insert({to, from});
    }
    _data[from][to] = diff; // we update the difference..
    _preds[to] = from;      // we update the predecessor..
}

void rdl_theory::resize(const size_t &size)
{
    const size_t c_size = _data.size();
    for (auto &row : _data)
        row.resize(size, rational::POSITIVE_INFINITY);
    _data.resize(size, std::vector<inf_rational>(size, rational::POSITIVE_INFINITY));
    for (size_t i = c_size; i < size; ++i)
        _data[i][i] = 0;
}
} // namespace smt
