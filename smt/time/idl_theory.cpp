#include "idl_theory.h"
#include <limits>
#include <algorithm>
#include <cassert>

namespace smt
{

idl_theory::idl_theory(sat_core &sat, const size_t &size) : theory(sat), _dists(std::vector<std::vector<I>>(size, std::vector<I>(size, std::numeric_limits<I>::max()))), _preds(std::vector<std::vector<var>>(size, std::vector<var>(size, -1)))
{
    for (size_t i = 0; i < size; ++i)
    {
        _dists[i][i] = 0;
        std::fill(_preds[i].begin(), _preds[i].end(), i);
        _preds[i][i] = -1;
    }
}

idl_theory::~idl_theory() {}

var idl_theory::new_var()
{
    var tp = n_vars++;
    if (_dists.size() == tp)
        resize((_dists.size() * 3) / 2 + 1);
    return tp;
}

var idl_theory::new_distance(const var &from, const var &to, const I &dist)
{
    if (_dists[to][from] < -dist)
        return FALSE_var; // the constraint is inconsistent..
    else if (_dists[from][to] <= dist)
        return TRUE_var; // the constraint is redundant..
    else
    { // we need to create a new propositional variable..
        const var ctr = sat.new_var();
        bind(ctr);
        idl_distance *c_dist = new idl_distance(ctr, from, to, dist);
        var_dists[ctr].push_back(c_dist);
        dist_constrs[{from, to}].push_back(c_dist);
        return ctr;
    }
}

bool idl_theory::distance(const var &from, const var &to, const I &dist, const var &p)
{
    if (_dists[to][from] < -dist)
        return sat.eq(p, FALSE_var); // the constraint is inconsistent..
    else if (_dists[from][to] <= dist)
        return sat.eq(p, TRUE_var); // the constraint is redundant..
    else
        switch (sat.value(p))
        {
        case True:
        {
            propagate(from, to, dist);
            return true;
        }
        case False:
        {
            propagate(to, from, -dist - 1);
            return true;
        }
        default:
            bind(p);
            idl_distance *c_dist = new idl_distance(p, from, to, dist);
            var_dists[p].push_back(c_dist);
            dist_constrs[{from, to}].push_back(c_dist);
            return true;
        }
}

bool idl_theory::propagate(const lit &p, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    assert(var_dists.count(p.get_var()));
    if (p.get_sign())
    { // the assertion is direct..
        for (const auto &dist : var_dists.at(p.get_var()))
        {
            if (_dists[dist->to][dist->from] < -dist->dist)
            { // we build the cause for the conflict..
                var c_to = dist->from;
                while (c_to != dist->to)
                {
                    if (const auto &c_d = dist_constrs.find({_preds[dist->to][c_to], c_to}); c_d != dist_constrs.end())
                        for (const auto &c_dist : c_d->second)
                            if (sat.value(c_dist->b) != Undefined)
                                cnfl.push_back(lit(c_dist->b, false));
                    c_to = _preds[dist->to][c_to];
                }
                cnfl.push_back(!p);
                return false;
            }

            if (_dists[dist->from][dist->to] > dist->dist)
                propagate(dist->from, dist->to, dist->dist);
        }
    }
    else
    { // the assertion is negated: we try semantic branching..
        for (const auto &dist : var_dists.at(p.get_var()))
        {
            if (_dists[dist->from][dist->to] <= dist->dist)
            { // we build the cause for the conflict..
                var c_from = dist->to;
                while (c_from != dist->from)
                {
                    if (const auto &c_d = dist_constrs.find({_preds[dist->from][c_from], c_from}); c_d != dist_constrs.end())
                        for (const auto &c_dist : c_d->second)
                            if (sat.value(c_dist->b) != Undefined)
                                cnfl.push_back(lit(c_dist->b, false));
                    c_from = _preds[dist->from][c_from];
                }
                cnfl.push_back(!p);
                return false;
            }

            if (_dists[dist->to][dist->from] >= -dist->dist)
                propagate(dist->to, dist->from, -dist->dist - 1);
        }
    }
    return true;
}

bool idl_theory::check(std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    return true;
}

void idl_theory::push() { layers.push_back(layer()); }

void idl_theory::pop()
{
    for (const auto &dist : layers.back().old_dists)
        _dists[dist.first.first][dist.first.second] = dist.second;
    for (const auto &pred : layers.back().old_preds)
        _preds[pred.first.first][pred.first.second] = pred.second;
    layers.pop_back();
}

void idl_theory::propagate(const var &from, const var &to, const I &dist)
{
    set_dist(from, to, dist);
    set_pred(from, to, from);
    std::vector<var> set_i;
    std::vector<var> set_j;
#ifdef STRONG_PROPAGATION
    std::vector<std::pair<var, var>> c_updates;
    c_updates.push_back({from, to});
    c_updates.push_back({to, from});
#endif

    // we start with an O(n) loop..
    for (size_t u = 0; u < size(); ++u)
    {
        if (_dists[u][from] < _dists[u][to] - dist)
        { // u -> from -> to is shorter than u -> to..
            set_dist(u, to, _dists[u][from] + dist);
            set_pred(u, to, from);
            set_i.push_back(u);
#ifdef STRONG_PROPAGATION
            c_updates.push_back({u, to});
            c_updates.push_back({to, u});
#endif
        }
        if (_dists[to][u] < _dists[from][u] - dist)
        { // from -> to -> u is shorter than from -> u..
            set_dist(from, u, _dists[to][u] + dist);
            set_pred(from, u, _preds[to][u]);
            set_j.push_back(u);
#ifdef STRONG_PROPAGATION
            c_updates.push_back({from, u});
            c_updates.push_back({u, from});
#endif
        }
    }

    // finally, we loop over set_i and set_j in O(n^2) time (but possibly much less)..
    for (const auto &i : set_i)
        for (const auto &j : set_j)
            if (i != j && _dists[i][to] + _dists[to][j] < _dists[i][j])
            { // i -> from -> to -> j is shorter than i -> j--
                set_dist(i, j, _dists[i][to] + _dists[to][j]);
                set_pred(i, j, _preds[to][j]);
#ifdef STRONG_PROPAGATION
                c_updates.push_back({i, j});
                c_updates.push_back({j, i});
#endif
            }

#ifdef STRONG_PROPAGATION
    for (const auto &c_pairs : c_updates)
        if (const auto &c_dists = dist_constrs.find(c_pairs); c_dists != dist_constrs.end())
            for (const auto &dist : c_dists->second)
                if (sat.value(dist->b) == Undefined)
                    if (_dists[dist->to][dist->from] < -dist->dist)
                    { // the constraint is inconsistent..
                        std::vector<lit> cnfl;
                        cnfl.push_back(lit(dist->b, false));
                        var c_to = dist->from;
                        while (c_to != dist->to)
                        {
                            if (const auto &c_d = dist_constrs.find({_preds[dist->to][c_to], c_to}); c_d != dist_constrs.end())
                                for (const auto &c_dist : c_d->second)
                                    if (sat.value(c_dist->b) != Undefined)
                                        cnfl.push_back(lit(c_dist->b, false));
                            c_to = _preds[dist->to][c_to];
                        }
                        // we propagate the reason for assigning false to dist->b..
                        record(cnfl);
                    }
                    else if (_dists[dist->from][dist->to] <= dist->dist)
                    { // the constraint is redundant..
                        std::vector<lit> cnfl;
                        cnfl.push_back(dist->b);
                        var c_to = dist->to;
                        while (c_to != dist->from)
                        {
                            if (const auto &c_d = dist_constrs.find({_preds[dist->from][c_to], c_to}); c_d != dist_constrs.end())
                                for (const auto &c_dist : c_d->second)
                                    if (sat.value(c_dist->b) != Undefined)
                                        cnfl.push_back(lit(c_dist->b, false));
                            c_to = _preds[dist->from][c_to];
                        }
                        // we propagate the reason for assigning true to dist->b..
                        record(cnfl);
                    }
#endif
}

void idl_theory::set_dist(const var &from, const var &to, const I &dist)
{
    assert(_dists[from][to] > dist);
    if (!layers.empty() && !layers.back().old_dists.count({from, to}))
        // we store the current values for backtracking purposes..
        layers.back().old_dists.insert({{from, to}, _dists[from][to]});
    // we update the disterence..
    _dists[from][to] = dist;
}

void idl_theory::set_pred(const var &from, const var &to, const var &pred)
{
    if (!layers.empty() && !layers.back().old_preds.count({from, to}))
        // we store the current values for backtracking purposes..
        layers.back().old_preds.insert({{from, to}, from});
    // we update the predecessor..
    _preds[from][to] = pred;
}

void idl_theory::resize(const size_t &size)
{
    const size_t c_size = _dists.size();
    for (auto &row : _dists)
        row.resize(size, std::numeric_limits<I>::max());
    _dists.resize(size, std::vector<I>(size, std::numeric_limits<I>::max()));
    for (size_t i = c_size; i < size; ++i)
        _dists[i][i] = 0;

    for (size_t i = c_size; i < size; ++i)
        _preds[i].resize(size, i);
    _preds.resize(size, std::vector<var>(size, -1));
    for (size_t i = c_size; i < size; ++i)
    {
        std::fill(_preds[i].begin(), _preds[i].end(), i);
        _preds[i][i] = -1;
    }
}
} // namespace smt
