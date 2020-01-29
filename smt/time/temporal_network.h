#pragma once

#include "sat_core.h"
#include "rational.h"
#include <limits>

namespace smt
{

template <typename T>
class temporal_network
{
public:
    temporal_network(const size_t &size) : _data(std::vector<std::vector<T>>(size, std::vector<T>(size, std::numeric_limits<T>::infinity())))
    {
        for (size_t i = 0; i < size; ++i)
            _data[i][i] = 0;
        add_constraint(origin(), horizon(), 0, std::numeric_limits<T>::infinity());
    }
    ~temporal_network() {}

    size_t new_tp()
    {
        size_t tp = n_vars++;
        if (_data.size() == tp)
            resize((_data.size() * 3) / 2 + 1);
        add_constraint(origin(), tp, 0, std::numeric_limits<T>::infinity());
        add_constraint(tp, horizon(), 0, std::numeric_limits<T>::infinity());
        return tp;
    }

    bool add_constraint(const size_t &from, const size_t &to, const T &min, const T &max)
    {
        if (_data[to][from] < -max || _data[from][to] < min)
            return false;
        if (_data[from][to] > max)
            propagate(from, to, max);
        if (_data[to][from] > -min)
            propagate(to, from, -min);
        return true;
    }

    std::pair<T, T> dist(const size_t &from, const size_t &to) const { return std::pair<T, T>(-_data[from][to], _data[to][from]); }
    std::pair<T, T> bound(const size_t &tp) const { return std::pair<T, T>(-_data[tp][origin()], _data[origin()][tp]); }
    T lb(const size_t &tp) const { return -_data[tp][origin()]; }
    T ub(const size_t &tp) const { return _data[origin()][tp]; }

    size_t size() const { return n_vars; }

private:
    void propagate(const size_t &from, const size_t &to, const T &weight)
    {
        _data[from][to] = weight;
        std::vector<size_t> set_i(n_vars, 0);
        std::vector<size_t> set_j(n_vars, 0);
        size_t size_i = 0, size_j = 0;

        // start with an O(n) loop
        for (size_t k = 0; k < n_vars; ++k)
        {
            if (_data[k][from] < _data[k][to] - weight)
            { // u -> from -> to is shorter than u -> to
                _data[k][to] = _data[k][from] + weight;
                set_i[size_i++] = k;
            }
            if (_data[to][k] < _data[from][k] - weight)
            { // from -> to -> u is shorter than from -> u
                _data[from][k] = weight + _data[to][k];
                set_j[size_j++] = k;
            }
        }

        // finally, loop over set_i and set_j in O(n^2) time (but possibly much less)
        for (int i = 0; i < size_i; ++i)
            for (int j = 0; j < size_j; ++j)
                if (set_i[i] != set_j[j] && _data[set_i[i]][to] + _data[to][set_j[j]] < _data[set_i[i]][set_j[j]])
                    // set_i[i] -> from -> to -> set_j[j] is shorter than set_i[i] -> set_j[j]
                    _data[set_i[i]][set_j[j]] = _data[set_i[i]][to] + _data[to][set_j[j]];
    }

    void resize(const size_t &size)
    {
        const size_t c_size = _data.size();
        for (auto &row : _data)
            row.resize(size, std::numeric_limits<T>::infinity());
        _data.resize(size, std::vector<T>(size, std::numeric_limits<T>::infinity()));
        for (size_t i = c_size; i < size; ++i)
            _data[i][i] = 0;
    }

public:
    static constexpr size_t origin() { return 0; }
    static constexpr size_t horizon() { return 1; }

private:
    size_t n_vars = 2;
    std::vector<std::vector<T>> _data;
};

template <>
class temporal_network<rational>
{
public:
    temporal_network(const size_t &size) : _data(std::vector<std::vector<rational>>(size, std::vector<rational>(size, rational::POSITIVE_INFINITY)))
    {
        for (size_t i = 0; i < size; ++i)
            _data[i][i] = 0;
        add_constraint(origin(), horizon(), 0, rational::POSITIVE_INFINITY);
    }
    ~temporal_network() {}

    size_t new_tp()
    {
        size_t tp = n_vars++;
        if (_data.size() == tp)
            resize((_data.size() * 3) / 2 + 1);
        add_constraint(origin(), tp, 0, rational::POSITIVE_INFINITY);
        add_constraint(tp, horizon(), 0, rational::POSITIVE_INFINITY);
        return tp;
    }

    bool add_constraint(const size_t &from, const size_t &to, const rational &min, const rational &max)
    {
        if (_data[to][from] < -max || _data[from][to] < min)
            return false;
        if (_data[from][to] > max)
            propagate(from, to, max);
        if (_data[to][from] > -min)
            propagate(to, from, -min);
        return true;
    }

    std::pair<rational, rational> dist(const size_t &from, const size_t &to) const { return std::pair<rational, rational>(-_data[from][to], _data[to][from]); }
    std::pair<rational, rational> bound(const size_t &tp) const { return std::pair<rational, rational>(-_data[tp][origin()], _data[origin()][tp]); }
    rational lb(const size_t &tp) const { return -_data[tp][origin()]; }
    rational ub(const size_t &tp) const { return _data[origin()][tp]; }

    size_t size() const { return n_vars; }

private:
    void propagate(const size_t &from, const size_t &to, const rational &weight)
    {
        _data[from][to] = weight;
        std::vector<size_t> set_i(n_vars, 0);
        std::vector<size_t> set_j(n_vars, 0);
        size_t size_i = 0, size_j = 0;

        // start with an O(n) loop
        for (size_t k = 0; k < n_vars; ++k)
        {
            if (_data[k][from] < _data[k][to] - weight)
            { // u -> from -> to is shorter than u -> to
                _data[k][to] = _data[k][from] + weight;
                set_i[size_i++] = k;
            }
            if (_data[to][k] < _data[from][k] - weight)
            { // from -> to -> u is shorter than from -> u
                _data[from][k] = weight + _data[to][k];
                set_j[size_j++] = k;
            }
        }

        // finally, loop over set_i and set_j in O(n^2) time (but possibly much less)
        for (int i = 0; i < size_i; ++i)
            for (int j = 0; j < size_j; ++j)
                if (set_i[i] != set_j[j] && _data[set_i[i]][to] + _data[to][set_j[j]] < _data[set_i[i]][set_j[j]])
                    // set_i[i] -> from -> to -> set_j[j] is shorter than set_i[i] -> set_j[j]
                    _data[set_i[i]][set_j[j]] = _data[set_i[i]][to] + _data[to][set_j[j]];
    }

    void resize(const size_t &size)
    {
        const size_t c_size = _data.size();
        for (auto &row : _data)
            row.resize(size, std::numeric_limits<rational>::infinity());
        _data.resize(size, std::vector<rational>(size, std::numeric_limits<rational>::infinity()));
        for (size_t i = c_size; i < size; ++i)
            _data[i][i] = 0;
    }

public:
    static constexpr size_t origin() { return 0; }
    static constexpr size_t horizon() { return 1; }

private:
    size_t n_vars = 2;
    std::vector<std::vector<rational>> _data;
};
} // namespace smt