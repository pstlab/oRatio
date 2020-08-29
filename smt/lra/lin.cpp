#include "lin.h"
#include <string>
#include <cassert>

namespace smt
{

    lin::lin() {}
    lin::lin(const rational &known_term) : known_term(known_term) {}
    lin::lin(const var v, const rational &c) { vars.emplace(v, c); }

    lin lin::operator+(const lin &right) const noexcept
    {
        lin res = *this;
        for (const auto &term : right.vars)
        {
            const auto trm_it = res.vars.find(term.first);
            if (trm_it == res.vars.end())
                res.vars.insert(term);
            else
            {
                trm_it->second += term.second;
                if (trm_it->second == rational::ZERO)
                    res.vars.erase(trm_it);
            }
        }
        res.known_term += known_term;
        return res;
    }

    lin lin::operator+(const rational &right) const noexcept
    {
        lin res = *this;
        res.known_term += right;
        return res;
    }

    lin operator+(const rational &lhs, const lin &rhs) noexcept
    {
        lin res = rhs;
        res.known_term += lhs;
        return res;
    }

    lin lin::operator-(const lin &right) const noexcept
    {
        lin res = *this;
        for (const auto &term : right.vars)
            if (const auto trm_it = res.vars.find(term.first); trm_it == res.vars.end())
                res.vars.emplace(term.first, -term.second);
            else
            {
                trm_it->second -= term.second;
                if (trm_it->second == rational::ZERO)
                    res.vars.erase(trm_it);
            }
        res.known_term -= right.known_term;
        return res;
    }

    lin lin::operator-(const rational &right) const noexcept
    {
        lin res = *this;
        res.known_term -= right;
        return res;
    }

    lin operator-(const rational &lhs, const lin &rhs) noexcept
    {
        lin res = -rhs;
        res.known_term += lhs;
        return res;
    }

    lin lin::operator*(const rational &right) const noexcept
    {
        lin res = *this;
        for (auto &&term : res.vars)
            term.second *= right;
        res.known_term *= right;
        return res;
    }

    lin operator*(const rational &lhs, const lin &rhs) noexcept
    {
        lin res = rhs;
        for (auto &&term : res.vars)
            term.second *= lhs;
        res.known_term *= lhs;
        return res;
    }

    lin lin::operator/(const rational &right) const noexcept
    {
        lin res = *this;
        for (auto &&term : res.vars)
            term.second /= right;
        res.known_term /= right;
        return res;
    }

    lin lin::operator+=(const lin &right) noexcept
    {
        for (const auto &term : right.vars)
            if (const auto trm_it = vars.find(term.first); trm_it == vars.end())
                vars.insert(term);
            else
            {
                trm_it->second += term.second;
                if (trm_it->second == rational::ZERO)
                    vars.erase(trm_it);
            }
        known_term += right.known_term;
        return *this;
    }

    lin lin::operator+=(const rational &right) noexcept
    {
        known_term += right;
        return *this;
    }

    lin lin::operator-=(const lin &right) noexcept
    {
        for (const auto &term : right.vars)
            if (const auto trm_it = vars.find(term.first); trm_it == vars.end())
                vars.emplace(term.first, -term.second);
            else
            {
                trm_it->second -= term.second;
                if (trm_it->second == rational::ZERO)
                    vars.erase(trm_it);
            }
        known_term -= right.known_term;
        return *this;
    }

    lin lin::operator-=(const rational &right) noexcept
    {
        known_term -= right;
        return *this;
    }

    lin lin::operator*=(const rational &right) noexcept
    {
        assert(!is_infinite(right));
        if (right == rational::ZERO)
        {
            vars.clear();
            known_term = 0;
        }
        else
            for (auto &&term : vars)
                term.second *= right;
        return *this;
    }

    lin lin::operator/=(const rational &right) noexcept
    {
        assert(right != rational::ZERO);
        if (is_infinite(right))
        {
            vars.clear();
            known_term = 0;
        }
        else
            for (auto &&term : vars)
                term.second /= right;
        known_term /= right;
        return *this;
    }

    lin lin::operator-() const noexcept
    {
        lin res;
        for (const auto &term : vars)
            res.vars.at(term.first) = -term.second;
        res.known_term = -known_term;
        return res;
    }

    std::string to_string(const lin &rhs) noexcept
    {
        if (rhs.vars.empty())
            return to_string(rhs.known_term);

        std::string s;
        for (std::map<const var, rational>::const_iterator it = rhs.vars.cbegin(); it != rhs.vars.cend(); ++it)
            if (it == rhs.vars.cbegin())
            {
                if (it->second == rational::ONE)
                    s += "x" + std::to_string(it->first);
                else if (it->second == -rational::ONE)
                    s += "-x" + std::to_string(it->first);
                else
                    s += to_string(it->second) + "*x" + std::to_string(it->first);
            }
            else
            {
                if (it->second == rational::ONE)
                    s += " + x" + std::to_string(it->first);
                else if (it->second == -rational::ONE)
                    s += " - x" + std::to_string(it->first);
                else if (is_positive(it->second))
                    s += " + " + to_string(it->second) + "*x" + std::to_string(it->first);
                else
                    s += " - " + to_string(-it->second) + "*x" + std::to_string(it->first);
            }

        if (is_positive(rhs.known_term))
            s += " + " + to_string(rhs.known_term);
        if (is_negative(rhs.known_term))
            s += " - " + to_string(-rhs.known_term);
        return s;
    }
} // namespace smt