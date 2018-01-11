#include "lin.h"
#include <string>
#include <cassert>

namespace smt
{

lin::lin() {}
lin::lin(const rational &known_term) : known_term(known_term) {}
lin::lin(const var v, const rational &c) { vars.insert({v, c}); }

lin lin::operator+(const lin &right) const
{
    lin res = *this;
    for (auto &term : right.vars)
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

lin lin::operator+(const rational &right) const
{
    lin res = *this;
    res.known_term += right;
    return res;
}

lin operator+(const rational &lhs, const lin &rhs)
{
    lin res = rhs;
    res.known_term += lhs;
    return res;
}

lin lin::operator-(const lin &right) const
{
    lin res = *this;
    for (auto &term : right.vars)
    {
        const auto trm_it = res.vars.find(term.first);
        if (trm_it == res.vars.end())
            res.vars.insert({term.first, -term.second});
        else
        {
            trm_it->second -= term.second;
            if (trm_it->second == rational::ZERO)
                res.vars.erase(trm_it);
        }
    }
    res.known_term -= right.known_term;
    return res;
}

lin lin::operator-(const rational &right) const
{
    lin res = *this;
    res.known_term -= right;
    return res;
}

lin operator-(const rational &lhs, const lin &rhs)
{
    lin res = -rhs;
    res.known_term += lhs;
    return res;
}

lin lin::operator*(const rational &right) const
{
    lin res = *this;
    for (auto &term : res.vars)
        term.second *= right;
    res.known_term *= right;
    return res;
}

lin operator*(const rational &lhs, const lin &rhs)
{
    lin res = rhs;
    for (auto &term : res.vars)
        term.second *= lhs;
    res.known_term *= lhs;
    return res;
}

lin lin::operator/(const rational &right) const
{
    lin res = *this;
    for (auto &term : res.vars)
        term.second /= right;
    res.known_term /= right;
    return res;
}

lin lin::operator+=(const lin &right)
{
    for (auto &term : right.vars)
    {
        const auto trm_it = vars.find(term.first);
        if (trm_it == vars.end())
            vars.insert(term);
        else
        {
            trm_it->second += term.second;
            if (trm_it->second == rational::ZERO)
                vars.erase(trm_it);
        }
    }
    known_term += right.known_term;
    return *this;
}

lin lin::operator+=(const rational &right)
{
    known_term += right;
    return *this;
}

lin lin::operator-=(const lin &right)
{
    for (auto &term : right.vars)
    {
        const auto trm_it = vars.find(term.first);
        if (trm_it == vars.end())
            vars.insert({term.first, -term.second});
        else
        {
            trm_it->second -= term.second;
            if (trm_it->second == rational::ZERO)
                vars.erase(trm_it);
        }
    }
    known_term -= right.known_term;
    return *this;
}

lin lin::operator-=(const rational &right)
{
    known_term -= right;
    return *this;
}

lin lin::operator*=(const rational &right)
{
    assert(!right.is_infinite());
    if (right == rational::ZERO)
    {
        vars.clear();
        known_term = 0;
    }
    else
        for (auto &term : vars)
            term.second *= right;
    return *this;
}

lin lin::operator/=(const rational &right)
{
    assert(right != rational::ZERO);
    if (right.is_infinite())
    {
        vars.clear();
        known_term = 0;
    }
    else
        for (auto &term : vars)
            term.second /= right;
    known_term /= right;
    return *this;
}

lin lin::operator-() const
{
    lin res;
    for (auto &term : vars)
        res.vars.at(term.first) = -term.second;
    res.known_term = -known_term;
    return res;
}

std::string lin::to_string() const
{
    if (vars.empty())
        return known_term.to_string();

    std::string s;
    for (std::map<const var, rational>::const_iterator it = vars.cbegin(); it != vars.cend(); ++it)
        if (it == vars.cbegin())
        {
            if (it->second == rational::ONE)
                s += "x" + std::to_string(it->first);
            else if (it->second == -rational::ONE)
                s += "-x" + std::to_string(it->first);
            else
                s += it->second.to_string() + "*x" + std::to_string(it->first);
        }
        else
        {
            if (it->second == rational::ONE)
                s += " + x" + std::to_string(it->first);
            else if (it->second == -rational::ONE)
                s += " - x" + std::to_string(it->first);
            else if (it->second.is_positive())
                s += " + " + it->second.to_string() + "*x" + std::to_string(it->first);
            else
                s += " - " + (-it->second).to_string() + "*x" + std::to_string(it->first);
        }

    if (known_term.is_positive())
        s += " + " + known_term.to_string();
    if (known_term.is_negative())
        s += " - " + (-known_term).to_string();
    return s;
}
}