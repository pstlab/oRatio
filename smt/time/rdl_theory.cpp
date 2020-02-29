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

size_t rdl_theory::new_var()
{
    size_t tp = _preds.size();
    if (_data.size() == tp)
        resize((_data.size() * 3) / 2 + 1);
    _preds.push_back(0);
    return tp;
}

var rdl_theory::new_difference(const var &from, const var &to, const inf_rational &diff)
{
    if ((_data[from][to] + diff).is_negative())
        return FALSE_var; // the constraint is inconsistent..
    else if ((_data[from][to] - diff).is_positive())
        return TRUE_var; // the constraint is redundant..
    else
    { // we need to create a new propositional variable..
        const var ctr = sat.new_var();
        bind(ctr);
        var_diffs[ctr].push_back(new rdl_difference(ctr, from, to, diff));
        return ctr;
    }
}

bool rdl_theory::difference(const var &from, const var &to, const inf_rational &diff, const var &p)
{
    if ((_data[from][to] + diff).is_negative())
        return sat.eq(p, FALSE_var); // the constraint is inconsistent..
    else if ((_data[from][to] - diff).is_positive())
        return sat.eq(p, TRUE_var); // the constraint is redundant..
    else
        switch (sat.value(p))
        {
        case True:
        {
            std::vector<lit> cnfl;
            return propagate(from, to, diff, p, cnfl);
        }
        case False:
        {
            std::vector<lit> cnfl;
            return propagate(to, from, -diff - inf_rational(rational::ZERO, rational::ONE), lit(p, false), cnfl);
        }
        default:
            bind(p);
            var_diffs[p].push_back(new rdl_difference(p, from, to, diff));
            return true;
        }
}

bool rdl_theory::propagate(const lit &p, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    if (p.get_sign())
    { // the assertion is direct..
        for (const auto &diff : var_diffs.at(p.get_var()))
            if (!propagate(diff->from, diff->to, diff->diff, p, cnfl))
                return false;
    }
    else
    { // the assertion is negated..
        for (const auto &diff : var_diffs.at(p.get_var()))
            if (!propagate(diff->to, diff->from, -diff->diff - inf_rational(rational::ZERO, rational::ONE), p, cnfl))
                return false;
    }

    for (const auto &ctr : var_diffs)
        for (const auto &diff : ctr.second)
            if (sat.value(diff->b) == Undefined)
                if ((_data[diff->from][diff->to] + diff->diff).is_negative())
                { // the constraint is inconsistent..
                    // TODO: propagate the reason for assigning false to diff->b..
                }
                else if ((_data[diff->from][diff->to] - diff->diff).is_positive())
                { // the constraint is redundant..
                    // TODO: propagate the reason for assigning true to diff->b..
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

bool rdl_theory::propagate(const size_t &from, const size_t &to, const inf_rational &diff, const lit &p, std::vector<lit> &cnfl)
{
    if ((_data[to][from] + diff).is_negative())
        // TODO: build the caouse for the conflict..
        return false;

    set_diff(from, to, diff);
    std::vector<size_t> set_i;
    std::vector<size_t> set_j;

    // start with an O(n) loop
    for (size_t u = 0; u < size(); ++u)
    {
        if (_data[u][from] < _data[u][to] - diff)
        { // u -> from -> to is shorter than u -> to
            set_diff(u, to, _data[u][from] + diff);
            set_i.push_back(u);
        }
        if (_data[to][u] < _data[from][u] - diff)
        { // from -> to -> u is shorter than from -> u
            set_diff(from, u, _data[to][u] + diff);
            set_j.push_back(u);
        }
    }

    // finally, loop over set_i and set_j in O(n^2) time (but possibly much less)
    for (int i = 0; i < set_i.size(); ++i)
        for (int j = 0; j < set_j.size(); ++j)
            if (set_i[i] != set_j[j] && _data[set_i[i]][to] + _data[to][set_j[j]] < _data[set_i[i]][set_j[j]])
                // set_i[i] -> from -> to -> set_j[j] is shorter than set_i[i] -> set_j[j]
                set_diff(set_i[i], set_j[j], _data[set_i[i]][to] + _data[to][set_j[j]]);
    return true;
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
    _data[from][to] = diff;
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
