#include "lin.h"
#include <string>
#include <cassert>

namespace smt
{
    SMT_EXPORT lin::lin() {}
    SMT_EXPORT lin::lin(const rational &known_term) : known_term(known_term) {}
    SMT_EXPORT lin::lin(const var v, const rational &c) { vars.emplace(v, c); }

    SMT_EXPORT lin lin::operator+(const lin &right) const noexcept
    {
        lin res = *this;
        for (const auto &term : right.vars)
        {
            const auto trm_it = res.vars.find(term.first);
            if (trm_it == res.vars.cend())
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

    SMT_EXPORT lin lin::operator+(const rational &right) const noexcept
    {
        lin res = *this;
        res.known_term += right;
        return res;
    }

    SMT_EXPORT lin operator+(const rational &lhs, const lin &rhs) noexcept
    {
        lin res = rhs;
        res.known_term += lhs;
        return res;
    }

    SMT_EXPORT lin lin::operator-(const lin &right) const noexcept
    {
        lin res = *this;
        for (const auto &term : right.vars)
            if (const auto trm_it = res.vars.find(term.first); trm_it == res.vars.cend())
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

    SMT_EXPORT lin lin::operator-(const rational &right) const noexcept
    {
        lin res = *this;
        res.known_term -= right;
        return res;
    }

    SMT_EXPORT lin operator-(const rational &lhs, const lin &rhs) noexcept
    {
        lin res = -rhs;
        res.known_term += lhs;
        return res;
    }

    SMT_EXPORT lin lin::operator*(const rational &right) const noexcept
    {
        lin res = *this;
        for (auto &[v, c] : res.vars)
            c *= right;
        res.known_term *= right;
        return res;
    }

    SMT_EXPORT lin operator*(const rational &lhs, const lin &rhs) noexcept
    {
        lin res = rhs;
        for (auto &[v, c] : res.vars)
            c *= lhs;
        res.known_term *= lhs;
        return res;
    }

    SMT_EXPORT lin lin::operator/(const rational &right) const noexcept
    {
        lin res = *this;
        for (auto &[v, c] : res.vars)
            c /= right;
        res.known_term /= right;
        return res;
    }

    SMT_EXPORT lin lin::operator+=(const lin &right) noexcept
    {
        for (const auto &term : right.vars)
            if (const auto trm_it = vars.find(term.first); trm_it == vars.cend())
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

    SMT_EXPORT lin lin::operator+=(const rational &right) noexcept
    {
        known_term += right;
        return *this;
    }

    SMT_EXPORT lin lin::operator-=(const lin &right) noexcept
    {
        for (const auto &[v, c] : right.vars)
            if (const auto trm_it = vars.find(v); trm_it == vars.cend())
                vars.emplace(v, -c);
            else
            {
                trm_it->second -= c;
                if (trm_it->second == rational::ZERO)
                    vars.erase(trm_it);
            }
        known_term -= right.known_term;
        return *this;
    }

    SMT_EXPORT lin lin::operator-=(const rational &right) noexcept
    {
        known_term -= right;
        return *this;
    }

    SMT_EXPORT lin lin::operator*=(const rational &right) noexcept
    {
        assert(!is_infinite(right));
        if (right == rational::ZERO)
        {
            vars.clear();
            known_term = rational::ZERO;
        }
        else
            for (auto &[v, c] : vars)
                c *= right;
        return *this;
    }

    SMT_EXPORT lin lin::operator/=(const rational &right) noexcept
    {
        assert(right != rational::ZERO);
        if (is_infinite(right))
        {
            vars.clear();
            known_term = rational::ZERO;
        }
        else
            for (auto &[v, c] : vars)
                c /= right;
        known_term /= right;
        return *this;
    }

    SMT_EXPORT lin lin::operator-() const noexcept
    {
        lin res;
        for (const auto &[v, c] : vars)
            res.vars.at(v) = -c;
        res.known_term = -known_term;
        return res;
    }

    SMT_EXPORT std::string to_string(const lin &rhs) noexcept
    {
        if (rhs.vars.empty())
            return to_string(rhs.known_term);

        std::string s;
        for (auto it = rhs.vars.cbegin(); it != rhs.vars.cend(); ++it)
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