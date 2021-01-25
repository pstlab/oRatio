#include "rdl_theory.h"
#include "rdl_value_listener.h"
#include <algorithm>
#include <stdexcept>
#include <cassert>

namespace smt
{

    rdl_theory::rdl_theory(sat_core &sat, const size_t &size) : theory(sat), _dists(std::vector<std::vector<inf_rational>>(size, std::vector<inf_rational>(size, inf_rational(rational::POSITIVE_INFINITY)))), _preds(std::vector<std::vector<var>>(size, std::vector<var>(size, -1)))
    {
        for (size_t i = 0; i < size; ++i)
        {
            _dists[i][i] = inf_rational(rational::ZERO);
            std::fill(_preds[i].begin(), _preds[i].end(), i);
            _preds[i][i] = -1;
        }
    }

    rdl_theory::~rdl_theory() {}

    var rdl_theory::new_var() noexcept
    {
        var tp = n_vars++;
        if (_dists.size() == tp)
            resize((_dists.size() * 3) / 2 + 1);
        return tp;
    }

    lit rdl_theory::new_distance(const var &from, const var &to, const inf_rational &dist) noexcept
    {
        if (_dists[to][from] < -dist)
            return FALSE_lit; // the constraint is inconsistent..
        else if (_dists[from][to] <= dist)
            return TRUE_lit; // the constraint is redundant..
        else
        { // we need to create a new propositional variable..
            const var ctr = sat.new_var();
            const lit ctr_lit(ctr);
            bind(ctr);
            const auto dst_cnst = new rdl_distance(ctr_lit, from, to, dist);
            var_dists[ctr].push_back(dst_cnst);
            dist_constrs[{from, to}].push_back(dst_cnst);
            return ctr_lit;
        }
    }

    lit rdl_theory::new_lt(const lin &left, const lin &right)
    {
        lin expr = left - right;
        switch (expr.vars.size())
        {
        case 0:
            return expr.known_term < rational::ZERO ? TRUE_lit : FALSE_lit;
        case 1:
            if (expr.vars.begin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.begin()->second;
                return new_distance(expr.vars.begin()->first, 0, inf_rational(expr.known_term, -1));
            }
            else
            {
                expr = expr / expr.vars.begin()->second;
                return new_distance(0, expr.vars.begin()->first, -inf_rational(expr.known_term, 1));
            }
        case 2:
            if (expr.vars.begin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.begin()->second;
                auto it = expr.vars.begin();
                const auto first_term = *it++;
                assert(first_term.second == rational::ONE);
                const auto second_term = *it;
                if (second_term.second != -rational::ONE)
                    throw std::invalid_argument("not a valid real difference logic constraint..");
                return new_distance(first_term.first, second_term.first, inf_rational(expr.known_term, -1));
            }
            else
            {
                expr = expr / expr.vars.begin()->second;
                auto it = expr.vars.begin();
                const auto first_term = *it++;
                assert(first_term.second == rational::ONE);
                const auto second_term = *it;
                if (second_term.second != -rational::ONE)
                    throw std::invalid_argument("not a valid real difference logic constraint..");
                return new_distance(second_term.first, first_term.first, -inf_rational(expr.known_term, 1));
            }
        default:
            throw std::invalid_argument("not a valid real difference logic constraint..");
        }
    }

    lit rdl_theory::new_leq(const lin &left, const lin &right)
    {
        lin expr = left - right;
        switch (expr.vars.size())
        {
        case 0:
            return expr.known_term <= rational::ZERO ? TRUE_lit : FALSE_lit;
        case 1:
            if (expr.vars.begin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.begin()->second;
                return new_distance(expr.vars.begin()->first, 0, inf_rational(expr.known_term));
            }
            else
            {
                expr = expr / expr.vars.begin()->second;
                return new_distance(0, expr.vars.begin()->first, -inf_rational(expr.known_term));
            }
        case 2:
            if (expr.vars.begin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.begin()->second;
                auto it = expr.vars.begin();
                const auto first_term = *it++;
                assert(first_term.second == rational::ONE);
                const auto second_term = *it;
                if (second_term.second != -rational::ONE)
                    throw std::invalid_argument("not a valid real difference logic constraint..");
                return new_distance(first_term.first, second_term.first, inf_rational(expr.known_term));
            }
            else
            {
                expr = expr / expr.vars.begin()->second;
                auto it = expr.vars.begin();
                const auto first_term = *it++;
                assert(first_term.second == rational::ONE);
                const auto second_term = *it;
                if (second_term.second != -rational::ONE)
                    throw std::invalid_argument("not a valid real difference logic constraint..");
                return new_distance(second_term.first, first_term.first, -inf_rational(expr.known_term));
            }
        default:
            throw std::invalid_argument("not a valid real difference logic constraint..");
        }
    }

    lit rdl_theory::new_eq(const lin &left, const lin &right)
    {
        lin expr = left - right;
        switch (expr.vars.size())
        {
        case 0:
            return expr.known_term == rational::ZERO ? TRUE_lit : FALSE_lit;
        case 1:
        {
            expr = expr / expr.vars.begin()->second;
            const auto dist = distance(expr.vars.begin()->first, 0);
            if (dist.first <= expr.known_term && dist.second >= expr.known_term)
                return sat.new_conj({new_distance(expr.vars.begin()->first, 0, inf_rational(expr.known_term)), new_distance(0, expr.vars.begin()->first, -inf_rational(expr.known_term))});
            else
                return FALSE_lit;
        }
        case 2:
        {
            expr = expr / expr.vars.begin()->second;
            auto it = expr.vars.begin();
            const auto first_term = *it++;
            assert(first_term.second == rational::ONE);
            const auto second_term = *it;
            if (second_term.second != -rational::ONE)
                throw std::invalid_argument("not a valid real difference logic constraint..");
            const auto dist = distance(first_term.first, second_term.first);
            if (dist.first <= expr.known_term && dist.second >= expr.known_term)
                return sat.new_conj({new_distance(first_term.first, second_term.first, inf_rational(expr.known_term)), new_distance(second_term.first, first_term.first, -inf_rational(expr.known_term))});
            else
                return FALSE_lit;
        }
        default:
            throw std::invalid_argument("not a valid real difference logic constraint..");
        }
    }

    lit rdl_theory::new_geq(const lin &left, const lin &right)
    {
        lin expr = left - right;
        switch (expr.vars.size())
        {
        case 0:
            return expr.known_term >= rational::ZERO ? TRUE_lit : FALSE_lit;
        case 1:
            if (expr.vars.begin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.begin()->second;
                return new_distance(0, expr.vars.begin()->first, -inf_rational(expr.known_term));
            }
            else
            {
                expr = expr / expr.vars.begin()->second;
                return new_distance(expr.vars.begin()->first, 0, inf_rational(expr.known_term));
            }
        case 2:
        {
            if (expr.vars.begin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.begin()->second;
                auto it = expr.vars.begin();
                const auto first_term = *it++;
                assert(first_term.second == rational::ONE);
                const auto second_term = *it;
                if (second_term.second != -rational::ONE)
                    throw std::invalid_argument("not a valid real difference logic constraint..");
                return new_distance(second_term.first, first_term.first, -inf_rational(expr.known_term));
            }
            else
            {
                expr = expr / expr.vars.begin()->second;
                auto it = expr.vars.begin();
                const auto first_term = *it++;
                assert(first_term.second == rational::ONE);
                const auto second_term = *it;
                if (second_term.second != -rational::ONE)
                    throw std::invalid_argument("not a valid real difference logic constraint..");
                return new_distance(first_term.first, second_term.first, inf_rational(expr.known_term));
            }
        }
        default:
            throw std::invalid_argument("not a valid real difference logic constraint..");
        }
    }

    lit rdl_theory::new_gt(const lin &left, const lin &right)
    {
        lin expr = left - right;
        switch (expr.vars.size())
        {
        case 0:
            return expr.known_term > rational::ZERO ? TRUE_lit : FALSE_lit;
        case 1:
            if (expr.vars.begin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.begin()->second;
                return new_distance(0, expr.vars.begin()->first, -inf_rational(expr.known_term, 1));
            }
            else
            {
                expr = expr / expr.vars.begin()->second;
                return new_distance(expr.vars.begin()->first, 0, inf_rational(expr.known_term, -1));
            }
        case 2:
            if (expr.vars.begin()->second < rational::ZERO)
            {
                expr = expr / expr.vars.begin()->second;
                auto it = expr.vars.begin();
                const auto first_term = *it++;
                assert(first_term.second == rational::ONE);
                const auto second_term = *it;
                if (second_term.second != -rational::ONE)
                    throw std::invalid_argument("not a valid real difference logic constraint..");
                return new_distance(second_term.first, first_term.first, -inf_rational(expr.known_term, 1));
            }
            else
            {
                expr = expr / expr.vars.begin()->second;
                auto it = expr.vars.begin();
                const auto first_term = *it++;
                assert(first_term.second == rational::ONE);
                const auto second_term = *it;
                if (second_term.second != -rational::ONE)
                    throw std::invalid_argument("not a valid real difference logic constraint..");
                return new_distance(first_term.first, second_term.first, inf_rational(expr.known_term, -1));
            }
        default:
            throw std::invalid_argument("not a valid real difference logic constraint..");
        }
    }

    std::pair<inf_rational, inf_rational> rdl_theory::bounds(const lin &l) const
    {
        inf_rational c_lb;
        inf_rational c_ub;

        switch (l.vars.size())
        {
        case 0:
            c_lb += l.known_term;
            c_ub += l.known_term;
            break;
        case 1:
        {
            std::map<const var, rational>::const_iterator it = l.vars.begin();
            c_lb += lb(it->first) * it->second + l.known_term;
            c_ub += ub(it->first) * it->second + l.known_term;
            break;
        }
        case 2:
        {
            const auto expr = l / l.vars.begin()->second;
            std::map<const var, rational>::const_iterator it = expr.vars.begin();
            const auto first_term = *it++;
            assert(first_term.second == rational::ONE);
            const auto second_term = *it;
            if (second_term.second != -rational::ONE)
                throw std::invalid_argument("not a valid real difference logic expression..");
            const auto dist = distance(first_term.first, second_term.first);
            c_lb += dist.first + expr.known_term;
            c_ub += dist.second + expr.known_term;
        }
        default:
            throw std::invalid_argument("not a valid real difference logic expression..");
        }
        return std::make_pair(c_lb, c_ub);
    }

    std::pair<inf_rational, inf_rational> rdl_theory::distance(const lin &from, const lin &to) const
    {
        lin expr = from - to;
        switch (expr.vars.size())
        {
        case 0:
            return std::make_pair(inf_rational(expr.known_term), inf_rational(expr.known_term));
        case 1:
        {
            expr = expr / expr.vars.begin()->second;
            return distance(expr.vars.begin()->first, 0);
        }
        case 2:
        {
            expr = expr / expr.vars.begin()->second;
            auto it = expr.vars.begin();
            const auto first_term = *it++;
            assert(first_term.second == rational::ONE);
            const auto second_term = *it;
            if (second_term.second != -rational::ONE)
                throw std::invalid_argument("not a valid real difference logic constraint..");
            return distance(first_term.first, second_term.first);
        }
        default:
            throw std::invalid_argument("not a valid real difference logic constraint..");
        }
    }

    bool rdl_theory::equates(const lin &l0, const lin &l1) const
    {
        if (l0.vars.empty() && l1.vars.empty())
            return l0.known_term == l1.known_term;
        else if (l0.vars.empty() && l1.vars.size() == 1)
        {
            const auto ae_bounds = bounds(l1);
            return ae_bounds.first <= l0.known_term && ae_bounds.second >= l0.known_term;
        }
        else if (l0.vars.size() == 1 && l1.vars.empty())
        {
            const auto l0_bounds = bounds(l0);
            return l0_bounds.first <= l1.known_term && l0_bounds.second >= l1.known_term;
        }
        else if (l0.vars.size() == 1 && l1.vars.size() == 1)
        {
            const auto dist = distance(l0.vars.begin()->first, l1.vars.begin()->first);
            const auto kt = l0.known_term - l1.known_term;
            return dist.first + kt <= 0 && dist.second + kt >= 0;
        }
        else
            throw std::invalid_argument("not a valid comparison between real difference logic expressions..");
    }

    bool rdl_theory::propagate(const lit &p) noexcept
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
                        if (const auto &c_d = dist_constr.find({_preds[dist->to][c_to], c_to}); c_d != dist_constr.end())
                            if (sat.value(c_d->second->b) == True)
                                cnfl.push_back(!c_d->second->b);
                            else if (sat.value(c_d->second->b) == False)
                                cnfl.push_back(c_d->second->b);
                        c_to = _preds[dist->to][c_to];
                    }
                    cnfl.push_back(!p);
                    return false;
                }
                else if (_dists[dist->from][dist->to] > dist->dist)
                { // we propagate..
                    const auto from_to = std::make_pair(dist->from, dist->to);
                    if (!layers.empty() && !layers.back().old_constrs.count(from_to))
                        if (const auto &c_dist = dist_constr.find(from_to); c_dist != dist_constr.end())
                            // we store the current constraint for backtracking purposes..
                            layers.back().old_constrs.emplace(c_dist->first, c_dist->second);
                        else
                            layers.back().old_constrs.emplace(from_to, nullptr);
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
                        if (const auto &c_d = dist_constr.find({_preds[dist->from][c_from], c_from}); c_d != dist_constr.end())
                            if (sat.value(c_d->second->b) == True)
                                cnfl.push_back(!c_d->second->b);
                            else if (sat.value(c_d->second->b) == False)
                                cnfl.push_back(c_d->second->b);
                        c_from = _preds[dist->from][c_from];
                    }
                    cnfl.push_back(!p);
                    return false;
                }
                else if (_dists[dist->to][dist->from] >= -dist->dist)
                { // we propagate..
                    const auto to_from = std::make_pair(dist->to, dist->from);
                    if (!layers.empty() && !layers.back().old_constrs.count(to_from))
                        if (const auto &c_dist = dist_constr.find(to_from); c_dist != dist_constr.end())
                            // we store the current constraint for backtracking purposes..
                            layers.back().old_constrs.emplace(c_dist->first, c_dist->second);
                        else
                            layers.back().old_constrs.emplace(to_from, nullptr);
                    dist_constr.emplace(to_from, dist);
                    propagate(dist->to, dist->from, -dist->dist - inf_rational(rational::ZERO, rational::ONE));
                }
                break;
            }
        return true;
    }

    bool rdl_theory::check() noexcept
    {
        assert(cnfl.empty());
        assert(std::all_of(dist_constr.begin(), dist_constr.end(), [this](const std::pair<std::pair<var, var>, rdl_distance *> &dist) {
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

    void rdl_theory::push() noexcept { layers.push_back(layer()); }

    void rdl_theory::pop() noexcept
    {
        for (const auto &dist : layers.back().old_dists)
            _dists[dist.first.first][dist.first.second] = dist.second;
        for (const auto &pred : layers.back().old_preds)
            _preds[pred.first.first][pred.first.second] = pred.second;
        for (const auto &dist : layers.back().old_constrs)
            if (dist.second) // we replace the current constraint..
                dist_constr.emplace(dist.first, dist.second);
            else // we make some cleanings..
                dist_constr.erase(dist.first);
        layers.pop_back();
    }

    void rdl_theory::propagate(const var &from, const var &to, const inf_rational &dist) noexcept
    {
        assert(!is_infinite(dist));
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
            if (_dists[u][from] < _dists[u][to] - dist)
            { // u -> from -> to is shorter than u -> to..
                set_dist(u, to, _dists[u][from] + dist);
                set_pred(u, to, from);
                set_i.push_back(u);
                c_updates.push_back({u, to});
                c_updates.push_back({to, u});
            }
            if (_dists[to][u] < _dists[from][u] - dist)
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
            if (const auto &c_dists = dist_constrs.find(c_pairs); c_dists != dist_constrs.end())
                for (const auto &c_dist : c_dists->second)
                    if (sat.value(c_dist->b) == Undefined)
                        if (_dists[c_dist->to][c_dist->from] < -c_dist->dist)
                        { // the constraint is inconsistent..
                            std::vector<lit> cnfl;
                            cnfl.push_back(!c_dist->b);
                            var c_to = c_dist->from;
                            while (c_to != c_dist->to)
                            {
                                if (const auto &c_d = dist_constr.find({_preds[c_dist->to][c_to], c_to}); c_d != dist_constr.end())
                                    if (sat.value(c_d->second->b) == True)
                                        cnfl.push_back(!c_d->second->b);
                                    else if (sat.value(c_d->second->b) == False)
                                        cnfl.push_back(c_d->second->b);
                                c_to = _preds[c_dist->to][c_to];
                            }
                            // we propagate the reason for assigning false to dist->b..
                            record(cnfl);
                        }
                        else if (_dists[c_dist->from][c_dist->to] <= c_dist->dist)
                        { // the constraint is redundant..
                            std::vector<lit> cnfl;
                            cnfl.push_back(c_dist->b);
                            var c_to = c_dist->to;
                            while (c_to != c_dist->from)
                            {
                                if (const auto &c_d = dist_constr.find({_preds[c_dist->from][c_to], c_to}); c_d != dist_constr.end())
                                    if (sat.value(c_d->second->b) == True)
                                        cnfl.push_back(!c_d->second->b);
                                    else if (sat.value(c_d->second->b) == False)
                                        cnfl.push_back(c_d->second->b);
                                c_to = _preds[c_dist->from][c_to];
                            }
                            // we propagate the reason for assigning true to dist->b..
                            record(cnfl);
                        }
    }

    void rdl_theory::set_dist(const var &from, const var &to, const inf_rational &dist) noexcept
    {
        assert(_dists[from][to] > dist);
        if (!layers.empty() && !layers.back().old_dists.count({from, to}))
            // we store the current values for backtracking purposes..
            layers.back().old_dists.insert({{from, to}, _dists[from][to]});
        // we update the disterence..
        _dists[from][to] = dist;

        if (from == 0)
        {
            if (const auto at_c_x = listening.find(to); at_c_x != listening.end())
                for (const auto &l : at_c_x->second)
                    l->rdl_value_change(to);
        }
        else if (to == 0)
        {
            if (const auto at_c_x = listening.find(from); at_c_x != listening.end())
                for (const auto &l : at_c_x->second)
                    l->rdl_value_change(from);
        }
    }

    void rdl_theory::set_pred(const var &from, const var &to, const var &pred) noexcept
    {
        if (!layers.empty() && !layers.back().old_preds.count({from, to}))
            // we store the current values for backtracking purposes..
            layers.back().old_preds.insert({{from, to}, from});
        // we update the predecessor..
        _preds[from][to] = pred;
    }

    void rdl_theory::resize(const size_t &size) noexcept
    {
        const size_t c_size = _dists.size();
        for (auto &row : _dists)
            row.resize(size, inf_rational(rational::POSITIVE_INFINITY));
        _dists.resize(size, std::vector<inf_rational>(size, inf_rational(rational::POSITIVE_INFINITY)));
        for (size_t i = c_size; i < size; ++i)
            _dists[i][i] = inf_rational(rational::ZERO);

        for (size_t i = 0; i < c_size; ++i)
            _preds[i].resize(size, i);
        _preds.resize(size, std::vector<var>(size, -1));
        for (size_t i = c_size; i < size; ++i)
        {
            std::fill(_preds[i].begin(), _preds[i].end(), i);
            _preds[i][i] = -1;
        }
    }
} // namespace smt