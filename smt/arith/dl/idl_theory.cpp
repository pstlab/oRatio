#include "idl_theory.h"
#include "idl_value_listener.h"
#include <algorithm>
#include <stdexcept>
#include <cassert>

namespace smt
{
    SMT_EXPORT idl_theory::idl_theory(sat_core &sat, const size_t &size) : theory(sat), _dists(std::vector<std::vector<I>>(size, std::vector<I>(size, inf()))), _preds(std::vector<std::vector<var>>(size, std::vector<var>(size, std::numeric_limits<size_t>::max())))
    {
        for (size_t i = 0; i < size; ++i)
        {
            _dists[i][i] = 0;
            std::fill(_preds[i].begin(), _preds[i].end(), i);
            _preds[i][i] = std::numeric_limits<size_t>::max();
        }
    }
    SMT_EXPORT idl_theory::~idl_theory() {}

    SMT_EXPORT var idl_theory::new_var() noexcept
    {
        var tp = n_vars++;
        if (_dists.size() == tp)
            resize((_dists.size() * 3) / 2 + 1);
        return tp;
    }

    SMT_EXPORT lit idl_theory::new_distance(const var &from, const var &to, const I &dist) noexcept
    {
        if (_dists[to][from] < -dist)
            return FALSE_lit; // the constraint is inconsistent..
        else if (_dists[from][to] <= dist)
            return TRUE_lit; // the constraint is redundant..
        else
        { // we need to create a new propositional variable..
            const auto ctr = sat.new_var();
            const lit ctr_lit(ctr);
            bind(ctr);
            const auto dst_cnst = new idl_distance(ctr_lit, from, to, dist);
            var_dists[ctr].push_back(dst_cnst);
            dist_constrs[{from, to}].push_back(dst_cnst);
            return ctr_lit;
        }
    }

    SMT_EXPORT lit idl_theory::new_lt(const lin &left, const lin &right)
    {
        lin expr = left - right;
        switch (expr.vars.size())
        {
        case 0:
            return expr.known_term < rational::ZERO ? TRUE_lit : FALSE_lit;
        case 1:
            if (expr.vars.cbegin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.cbegin()->second;
                if (!is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(expr.vars.cbegin()->first, 0, expr.known_term.numerator() - 1);
            }
            else
            {
                expr = expr / expr.vars.cbegin()->second;
                if (!is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(0, expr.vars.cbegin()->first, -expr.known_term.numerator() - 1);
            }
        case 2:
            if (expr.vars.cbegin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.cbegin()->second;
                auto it = expr.vars.cbegin();
                const auto [v0, c0] = *it++;
                assert(c0 == rational::ONE);
                const auto [v1, c1] = *it;
                if (c1 != -rational::ONE || !is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(v0, v1, expr.known_term.numerator() - 1);
            }
            else
            {
                expr = expr / expr.vars.cbegin()->second;
                auto it = expr.vars.cbegin();
                const auto [v0, c0] = *it++;
                assert(c0 == rational::ONE);
                const auto [v1, c1] = *it;
                if (c1 != -rational::ONE || !is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(v1, v0, -expr.known_term.numerator() - 1);
            }
        default:
            throw std::invalid_argument("not a valid integer difference logic constraint..");
        }
    }

    SMT_EXPORT lit idl_theory::new_leq(const lin &left, const lin &right)
    {
        lin expr = left - right;
        switch (expr.vars.size())
        {
        case 0:
            return expr.known_term <= rational::ZERO ? TRUE_lit : FALSE_lit;
        case 1:
            if (expr.vars.cbegin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.cbegin()->second;
                if (!is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(expr.vars.cbegin()->first, 0, expr.known_term.numerator());
            }
            else
            {
                expr = expr / expr.vars.cbegin()->second;
                if (!is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(0, expr.vars.cbegin()->first, -expr.known_term.numerator());
            }
        case 2:
            if (expr.vars.cbegin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.cbegin()->second;
                auto it = expr.vars.cbegin();
                const auto [v0, c0] = *it++;
                assert(c0 == rational::ONE);
                const auto [v1, c1] = *it;
                if (c1 != -rational::ONE || !is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(v0, v1, expr.known_term.numerator());
            }
            else
            {
                expr = expr / expr.vars.cbegin()->second;
                auto it = expr.vars.cbegin();
                const auto [v0, c0] = *it++;
                assert(v0 == rational::ONE);
                const auto [v1, c1] = *it;
                if (c1 != -rational::ONE || !is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(v1, v0, -expr.known_term.numerator());
            }
        default:
            throw std::invalid_argument("not a valid integer difference logic constraint..");
        }
    }

    SMT_EXPORT lit idl_theory::new_eq(const lin &left, const lin &right)
    {
        lin expr = left - right;
        switch (expr.vars.size())
        {
        case 0:
            return expr.known_term == rational::ZERO ? TRUE_lit : FALSE_lit;
        case 1:
        {
            expr = expr / expr.vars.cbegin()->second;
            if (!is_integer(expr.known_term))
                throw std::invalid_argument("not a valid integer difference logic constraint..");
            const auto dist = distance(expr.vars.cbegin()->first, 0);
            if (dist.first <= expr.known_term.numerator() && dist.second >= expr.known_term.numerator())
                return sat.new_conj({new_distance(expr.vars.cbegin()->first, 0, expr.known_term.numerator()), new_distance(0, expr.vars.cbegin()->first, -expr.known_term.numerator())});
            else
                return FALSE_lit;
        }
        case 2:
        {
            expr = expr / expr.vars.cbegin()->second;
            auto it = expr.vars.cbegin();
            const auto [v0, c0] = *it++;
            assert(c0 == rational::ONE);
            const auto [v1, c1] = *it;
            if (c1 != -rational::ONE || !is_integer(expr.known_term))
                throw std::invalid_argument("not a valid integer difference logic constraint..");
            const auto dist = distance(v0, v1);
            if (dist.first <= expr.known_term.numerator() && dist.second >= expr.known_term.numerator())
                return sat.new_conj({new_distance(v0, v1, expr.known_term.numerator()), new_distance(v1, v0, -expr.known_term.numerator())});
            else
                return FALSE_lit;
        }
        default:
            throw std::invalid_argument("not a valid integer difference logic constraint..");
        }
    }

    SMT_EXPORT lit idl_theory::new_geq(const lin &left, const lin &right)
    {
        lin expr = left - right;
        switch (expr.vars.size())
        {
        case 0:
            return expr.known_term >= rational::ZERO ? TRUE_lit : FALSE_lit;
        case 1:
            if (expr.vars.cbegin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.cbegin()->second;
                if (!is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(0, expr.vars.cbegin()->first, -expr.known_term.numerator());
            }
            else
            {
                expr = expr / expr.vars.cbegin()->second;
                if (!is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(expr.vars.cbegin()->first, 0, expr.known_term.numerator());
            }
        case 2:
            if (expr.vars.cbegin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.cbegin()->second;
                auto it = expr.vars.cbegin();
                const auto [v0, c0] = *it++;
                assert(c0 == rational::ONE);
                const auto [v1, c1] = *it;
                if (c1 != -rational::ONE || !is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(v1, v0, -expr.known_term.numerator());
            }
            else
            {
                expr = expr / expr.vars.cbegin()->second;
                auto it = expr.vars.cbegin();
                const auto [v0, c0] = *it++;
                assert(c0 == rational::ONE);
                const auto [v1, c1] = *it;
                if (c1 != -rational::ONE || !is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(v0, v1, expr.known_term.numerator());
            }
        default:
            throw std::invalid_argument("not a valid integer difference logic constraint..");
        }
    }

    lit idl_theory::new_gt(const lin &left, const lin &right)
    {
        lin expr = left - right;
        switch (expr.vars.size())
        {
        case 0:
            return expr.known_term > rational::ZERO ? TRUE_lit : FALSE_lit;
        case 1:
            if (expr.vars.cbegin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.cbegin()->second;
                if (!is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(0, expr.vars.cbegin()->first, -expr.known_term.numerator() - 1);
            }
            else
            {
                expr = expr / expr.vars.cbegin()->second;
                if (!is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(expr.vars.cbegin()->first, 0, expr.known_term.numerator() - 1);
            }
        case 2:
            if (expr.vars.cbegin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.cbegin()->second;
                auto it = expr.vars.cbegin();
                const auto [v0, c0] = *it++;
                assert(c0 == rational::ONE);
                const auto [v1, c1] = *it;
                if (c1 != -rational::ONE || !is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(v1, v0, -expr.known_term.numerator() - 1);
            }
            else
            {
                expr = expr / expr.vars.cbegin()->second;
                auto it = expr.vars.cbegin();
                const auto [v0, c0] = *it++;
                assert(c0 == rational::ONE);
                const auto [v1, c1] = *it;
                if (c1 != -rational::ONE || !is_integer(expr.known_term))
                    throw std::invalid_argument("not a valid integer difference logic constraint..");
                return new_distance(v0, v1, expr.known_term.numerator(), -1);
            }
        default:
            throw std::invalid_argument("not a valid integer difference logic constraint..");
        }
    }

    SMT_EXPORT std::pair<I, I> idl_theory::bounds(const lin &l) const
    {
        I c_lb(0);
        I c_ub(0);

        switch (l.vars.size())
        {
        case 0:
            if (!is_integer(l.known_term))
                throw std::invalid_argument("not a valid integer difference logic constraint..");
            c_lb += l.known_term.numerator();
            c_ub += l.known_term.numerator();
            break;
        case 1:
        {
            auto it = l.vars.cbegin();
            if (!is_integer(it->second) | !is_integer(l.known_term))
                throw std::invalid_argument("not a valid integer difference logic constraint..");
            c_lb += lb(it->first) * it->second.numerator() + l.known_term.numerator();
            c_ub += ub(it->first) * it->second.numerator() + l.known_term.numerator();
            break;
        }
        case 2:
        {
            const auto expr = l / l.vars.cbegin()->second;
            auto it = expr.vars.cbegin();
            const auto [v0, c0] = *it++;
            const auto [v1, c1] = *it;
            if (!is_integer(c1) || c1.numerator() != -1 || !is_integer(l.known_term))
                throw std::invalid_argument("not a valid integer difference logic expression..");
            const auto dist = distance(v1, v0);
            c_lb += dist.first + expr.known_term.numerator();
            c_ub += dist.second + expr.known_term.numerator();
            break;
        }
        default:
            throw std::invalid_argument("not a valid integer difference logic expression..");
        }
        return std::make_pair(c_lb, c_ub);
    }

    SMT_EXPORT std::pair<I, I> idl_theory::distance(const lin &from, const lin &to) const
    {
        lin expr = from - to;
        switch (expr.vars.size())
        {
        case 0:
            return std::make_pair(expr.known_term.numerator(), expr.known_term.numerator());
        case 1:
        {
            expr = expr / expr.vars.cbegin()->second;
            if (!is_integer(expr.known_term))
                throw std::invalid_argument("not a valid integer difference logic constraint..");
            return distance(expr.vars.cbegin()->first, 0);
        }
        case 2:
        {
            expr = expr / expr.vars.cbegin()->second;
            auto it = expr.vars.cbegin();
            const auto [v0, c0] = *it++;
            assert(c0 == rational::ONE);
            const auto [v1, c1] = *it;
            if (c1 != -rational::ONE || !is_integer(expr.known_term))
                throw std::invalid_argument("not a valid real difference logic constraint..");
            return distance(v0, v1);
        }
        default:
            throw std::invalid_argument("not a valid real difference logic constraint..");
        }
    }

    SMT_EXPORT bool idl_theory::equates(const lin &l0, const lin &l1) const
    {
        if (l0.vars.empty() && l1.vars.empty())
            return l0.known_term == l1.known_term;
        else if (l0.vars.empty() && l1.vars.size() == 1)
        {
            const auto [lb, ub] = bounds(l1);
            return lb <= l0.known_term && ub >= l0.known_term;
        }
        else if (l0.vars.size() == 1 && l1.vars.empty())
        {
            const auto [lb, ub] = bounds(l0);
            return lb <= l1.known_term && ub >= l1.known_term;
        }
        else if (l0.vars.size() == 1 && l1.vars.size() == 1)
        {
            const auto [lb, ub] = distance(l0.vars.cbegin()->first, l1.vars.cbegin()->first);
            const auto kt = l0.known_term - l1.known_term;
            return lb + kt <= rational::ZERO && ub + kt >= rational::ZERO;
        }
        else
            throw std::invalid_argument("not a valid comparison between real difference logic expressions..");
    }

    bool idl_theory::propagate(const lit &p) noexcept
    {
        assert(cnfl.empty());
        assert(var_dists.count(variable(p)));

        for (const auto &dist : var_dists.at(variable(p)))
            switch (sat.value(dist->b))
            {
            case True: // the assertion is direct..
                if (_dists[dist->to][dist->from] < -dist->dist)
                { // we build the cause for the conflict..
                    var c_to = dist->from;
                    while (c_to != dist->to)
                    {
                        if (const auto &c_d = dist_constr.find({_preds[dist->to][c_to], c_to}); c_d != dist_constr.cend())
                        {
                            if (sat.value(c_d->second->b) == True)
                                cnfl.push_back(!c_d->second->b);
                            else if (sat.value(c_d->second->b) == False)
                                cnfl.push_back(c_d->second->b);
                        }
                        c_to = _preds[dist->to][c_to];
                    }
                    cnfl.push_back(!p);
                    return false;
                }
                else if (_dists[dist->from][dist->to] > dist->dist)
                { // we propagate..
                    const auto from_to = std::make_pair(dist->from, dist->to);
                    if (!layers.empty() && !layers.back().old_constrs.count(from_to))
                    {
                        if (const auto &c_dist = dist_constr.find(from_to); c_dist != dist_constr.cend())
                            // we store the current constraint for backtracking purposes..
                            layers.back().old_constrs.emplace(c_dist->first, c_dist->second);
                        else
                            layers.back().old_constrs.emplace(from_to, nullptr);
                    }
                    dist_constr.emplace(from_to, dist);
                    propagate(dist->from, dist->to, dist->dist);
                }
                break;
            case False: // the assertion is negated (semantic branching)..
                if (_dists[dist->from][dist->to] <= dist->dist)
                { // we build the cause for the conflict..
                    var c_from = dist->to;
                    while (c_from != dist->from)
                    {
                        if (const auto &c_d = dist_constr.find({_preds[dist->from][c_from], c_from}); c_d != dist_constr.cend())
                        {
                            if (sat.value(c_d->second->b) == True)
                                cnfl.push_back(!c_d->second->b);
                            else if (sat.value(c_d->second->b) == False)
                                cnfl.push_back(c_d->second->b);
                        }
                        c_from = _preds[dist->from][c_from];
                    }
                    cnfl.push_back(!p);
                    return false;
                }
                else if (_dists[dist->to][dist->from] >= -dist->dist)
                { // we propagate..
                    const auto to_from = std::make_pair(dist->to, dist->from);
                    if (!layers.empty() && !layers.back().old_constrs.count(to_from))
                    {
                        if (const auto &c_dist = dist_constr.find(to_from); c_dist != dist_constr.cend())
                            // we store the current constraint for backtracking purposes..
                            layers.back().old_constrs.emplace(c_dist->first, c_dist->second);
                        else
                            layers.back().old_constrs.emplace(to_from, nullptr);
                    }
                    dist_constr.emplace(to_from, dist);
                    propagate(dist->to, dist->from, -dist->dist - 1);
                }
                break;
            }
        return true;
    }

    bool idl_theory::check() noexcept
    {
        assert(cnfl.empty());
        assert(std::all_of(dist_constr.cbegin(), dist_constr.cend(), [this](const auto &dist)
                           {
                               switch (sat.value(dist.second->b))
                               {
                               case True: // the constraint is asserted..
                                   return _dists[dist.second->from][dist.second->to] <= dist.second->dist;
                               case False: // the constraint is negated..
                                   return _dists[dist.second->to][dist.second->from] < -dist.second->dist;
                               default: // the constraint is not asserted..
                                   return true;
                               }
                           }));
        return true;
    }

    void idl_theory::push() noexcept { layers.push_back(layer()); }

    void idl_theory::pop() noexcept
    {
        for (const auto &[vars, dist] : layers.back().old_dists)
            _dists[vars.first][vars.second] = dist;
        for (const auto &[vars, pred] : layers.back().old_preds)
            _preds[vars.first][vars.second] = pred;
        for (const auto &[vars, dist] : layers.back().old_constrs)
            if (dist) // we replace the current constraint..
                dist_constr.emplace(vars, dist);
            else // we make some cleanings..
                dist_constr.erase(vars);
        layers.pop_back();
    }

    void idl_theory::propagate(const var &from, const var &to, const I &dist) noexcept
    {
        assert(abs(dist) < inf());
        set_dist(from, to, dist);
        set_pred(from, to, from);
        std::vector<var> set_i;
        std::vector<var> set_j;
        std::vector<std::pair<var, var>> c_updates;
        c_updates.push_back({from, to});
        c_updates.push_back({to, from});

        // we start with an O(n) loop..
        for (size_t u = 0; u < size(); ++u)
        {
            if (_dists[u][from] != inf() && _dists[u][from] < _dists[u][to] - dist)
            { // u -> from -> to is shorter than u -> to..
                set_dist(u, to, _dists[u][from] + dist);
                set_pred(u, to, from);
                set_i.push_back(u);
                c_updates.push_back({u, to});
                c_updates.push_back({to, u});
            }
            if (_dists[to][u] != inf() && _dists[to][u] < _dists[from][u] - dist)
            { // from -> to -> u is shorter than from -> u..
                set_dist(from, u, _dists[to][u] + dist);
                set_pred(from, u, _preds[to][u]);
                set_j.push_back(u);
                c_updates.push_back({from, u});
                c_updates.push_back({u, from});
            }
        }

        // finally, we loop over set_i and set_j in O(n^2) time (but possibly much less)..
        for (const auto &i : set_i)
            for (const auto &j : set_j)
                if (i != j && _dists[i][to] + _dists[to][j] < _dists[i][j])
                { // i -> from -> to -> j is shorter than i -> j--
                    set_dist(i, j, _dists[i][to] + _dists[to][j]);
                    set_pred(i, j, _preds[to][j]);
                    c_updates.push_back({i, j});
                    c_updates.push_back({j, i});
                }

        for (const auto &c_pairs : c_updates)
            if (const auto &c_dists = dist_constrs.find(c_pairs); c_dists != dist_constrs.cend())
                for (const auto &c_dist : c_dists->second)
                    if (sat.value(c_dist->b) == Undefined)
                        if (_dists[c_dist->to][c_dist->from] < -c_dist->dist)
                        { // the constraint is inconsistent..
                            cnfl.push_back(!c_dist->b);
                            var c_to = c_dist->from;
                            while (c_to != c_dist->to)
                            {
                                if (const auto &c_d = dist_constr.find({_preds[c_dist->to][c_to], c_to}); c_d != dist_constr.cend())
                                    if (sat.value(c_d->second->b) == True)
                                        cnfl.push_back(!c_d->second->b);
                                    else if (sat.value(c_d->second->b) == False)
                                        cnfl.push_back(c_d->second->b);
                                c_to = _preds[c_dist->to][c_to];
                            }
                            // we propagate the reason for assigning false to dist->b..
                            record(cnfl);
                            cnfl.clear();
                        }
                        else if (_dists[c_dist->from][c_dist->to] <= c_dist->dist)
                        { // the constraint is redundant..
                            cnfl.push_back(c_dist->b);
                            var c_to = c_dist->to;
                            while (c_to != c_dist->from)
                            {
                                if (const auto &c_d = dist_constr.find({_preds[c_dist->from][c_to], c_to}); c_d != dist_constr.cend())
                                    if (sat.value(c_d->second->b) == True)
                                        cnfl.push_back(!c_d->second->b);
                                    else if (sat.value(c_d->second->b) == False)
                                        cnfl.push_back(c_d->second->b);
                                c_to = _preds[c_dist->from][c_to];
                            }
                            // we propagate the reason for assigning true to dist->b..
                            record(cnfl);
                            cnfl.clear();
                        }
    }

    void idl_theory::set_dist(const var &from, const var &to, const I &dist) noexcept
    {
        assert(_dists[from][to] > dist);
        if (!layers.empty() && !layers.back().old_dists.count({from, to}))
            // we store the current values for backtracking purposes..
            layers.back().old_dists.insert({{from, to}, _dists[from][to]});
        // we update the disterence..
        _dists[from][to] = dist;

        if (from == 0)
        {
            if (const auto at_c_x = listening.find(to); at_c_x != listening.cend())
                for (const auto &l : at_c_x->second)
                    l->idl_value_change(to);
        }
        else if (to == 0)
        {
            if (const auto at_c_x = listening.find(from); at_c_x != listening.cend())
                for (const auto &l : at_c_x->second)
                    l->idl_value_change(from);
        }
    }

    void idl_theory::set_pred(const var &from, const var &to, const var &pred) noexcept
    {
        if (!layers.empty() && !layers.back().old_preds.count({from, to}))
            // we store the current values for backtracking purposes..
            layers.back().old_preds.insert({{from, to}, from});
        // we update the predecessor..
        _preds[from][to] = pred;
    }

    void idl_theory::resize(const size_t &size) noexcept
    {
        const size_t c_size = _dists.size();
        for (auto &row : _dists)
            row.resize(size, inf());
        _dists.resize(size, std::vector<I>(size, inf()));
        for (size_t i = c_size; i < size; ++i)
            _dists[i][i] = 0;

        for (size_t i = 0; i < c_size; ++i)
            _preds[i].resize(size, i);
        _preds.resize(size, std::vector<var>(size, std::numeric_limits<size_t>::max()));
        for (size_t i = c_size; i < size; ++i)
        {
            std::fill(_preds[i].begin(), _preds[i].end(), i);
            _preds[i][i] = std::numeric_limits<size_t>::max();
        }
    }
} // namespace smt