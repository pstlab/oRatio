#include "sat_core.h"
#include "clause.h"
#include "sat_value_listener.h"
#include <algorithm>
#include <cmath>
#include <cassert>

namespace smt
{

sat_core::sat_core()
{
    var c_false = new_var(); // the false constant..
    var c_true = new_var();  // the true constant..
    assert(c_false == FALSE_var);
    assert(c_true == TRUE_var);
    assigns[FALSE_var] = False;
    assigns[TRUE_var] = True;
    level[FALSE_var] = 0;
    level[TRUE_var] = 0;
}

sat_core::~sat_core()
{
    // we delete all the constraints..
    for (const auto &c : constrs)
        delete c;
}

var sat_core::new_var()
{
    const var id = assigns.size();
    watches.push_back(std::vector<clause *>());
    watches.push_back(std::vector<clause *>());
    assigns.push_back(Undefined);
    exprs.emplace("b" + std::to_string(id), id);
    level.push_back(0);
    reason.push_back(nullptr);
    return id;
}

bool sat_core::new_clause(const std::vector<lit> &lits)
{
    assert(root_level());
    // we check if the clause is already satisfied and filter out false/duplicate literals..
    std::vector<lit> c_lits = lits;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    lit p;
    size_t j = 0;
    for (std::vector<lit>::const_iterator it = c_lits.begin(); it != c_lits.end(); ++it)
        if (value(*it) == True || *it == !p)
            return true; // the clause is already satisfied or represents a tautology..
        else if (value(*it) != False && *it != p)
        { // we need to include this literal in the clause..
            p = *it;
            c_lits[j++] = p;
        }
    c_lits.resize(j);

    switch (c_lits.size())
    {
    case 0: // the clause is unsatisfable..
        return false;
    case 1: // the clause is unique under the current assignment..
        return enqueue(c_lits[0]);
    default: // we need to create a new clause..
        constrs.push_back(new clause(*this, c_lits));
        return true;
    }
}

var sat_core::new_eq(const lit &left, const lit &right)
{
    assert(root_level());
    // we try to avoid creating a new variable..
    switch (value(left))
    {
    case True:
        switch (value(right))
        {
        case True:
            return TRUE_var; // the variables assume the same value..
        case False:
            return FALSE_var; // the variables cannot assume the same value..
        case Undefined:
            if (right.get_sign())
                return right.get_var();
        }
    case False:
        switch (value(right))
        {
        case True:
            return FALSE_var; // the variables cannot assume the same value..
        case False:
            return TRUE_var; // the variables assume the same value..
        case Undefined:
            if (!right.get_sign())
                return right.get_var();
        }
    case Undefined:
        switch (value(right))
        {
        case True:
            if (left.get_sign())
                return left.get_var();
        case False:
            if (!left.get_sign())
                return left.get_var();
        case Undefined:
            break;
        }
    }

    const std::string s_expr = "=" + (left.get_var() < right.get_var()) ? (to_string(left) + to_string(right)) : to_string(right) + to_string(left);
    if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    { // we need to create a new variable..
        const var e = new_var();
        bool nc = eq(left, right, e);
        assert(nc);
        return e;
    }
}

var sat_core::new_conj(const std::vector<lit> &ls)
{
    assert(root_level());
    // we try to avoid creating a new variable..
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    lit p;
    size_t j = 0;
    std::string s_expr = "&";
    for (std::vector<lit>::const_iterator it = c_lits.begin(); it != c_lits.end(); ++it)
        if (value(*it) == False || *it == !p)
            return FALSE_var; // the conjunction cannot be satisfied..
        else if (value(*it) != True && *it != p)
        { // we need to include this literal in the conjunction..
            p = *it;
            s_expr += to_string(p);
            c_lits[j++] = p;
        }
    c_lits.resize(j);

    if (c_lits.empty()) // an empty conjunction is assumed to be satisfied..
        return TRUE_var;
    else if (c_lits.size() == 1 && c_lits[0].get_sign())
        return c_lits[0].get_var();
    else if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    { // we need to create a new variable..
        const var c = new_var();
        bool nc = conj(c_lits, c);
        assert(nc);
        return c;
    }
}

var sat_core::new_disj(const std::vector<lit> &ls)
{
    assert(root_level());
    // we try to avoid creating a new variable..
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    lit p;
    size_t j = 0;
    std::string s_expr = "|";
    for (std::vector<lit>::const_iterator it = c_lits.begin(); it != c_lits.end(); ++it)
        if (value(*it) == True || *it == !p)
            return TRUE_var; // the disjunction is already satisfied..
        else if (value(*it) != False && *it != p)
        { // we need to include this literal in the conjunction..
            p = *it;
            s_expr += to_string(p);
            c_lits[j++] = p;
        }
    c_lits.resize(j);

    if (c_lits.empty()) // an empty disjunction is assumed to be unsatisfable..
        return FALSE_var;
    else if (c_lits.size() == 1 && c_lits[0].get_sign())
        return c_lits[0].get_var();
    else if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    { // we need to create a new variable..
        const var d = new_var();
        bool nc = disj(c_lits, d);
        assert(nc);
        return d;
    }
}

var sat_core::new_at_most_one(const std::vector<lit> &ls)
{
    assert(root_level());
    // we try to avoid creating a new variable..
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    lit p;
    size_t j = 0;
    std::string s_expr = "amo";
    for (std::vector<lit>::const_iterator it0 = c_lits.begin(); it0 != c_lits.end(); ++it0)
        if (value(*it0) == True)
            for (std::vector<lit>::const_iterator it1 = it0 + 1; it1 != c_lits.end(); ++it1)
            {
                if (value(*it1) == True || *it1 == !p)
                    return FALSE_var; // the at-most-one cannot be satisfied..
                else if (value(*it1) != False && *it1 != p)
                { // we need to include this literal in the exact-one..
                    p = *it1;
                    s_expr += to_string(p);
                    c_lits[j++] = p;
                }
            }
        else if (value(*it0) != False && *it0 != p)
        { // we need to include this literal in the at-most-one..
            p = *it0;
            s_expr += to_string(p);
            c_lits[j++] = p;
        }
    c_lits.resize(j);

    if (c_lits.empty() || c_lits.size() == 1) // an empty or a singleton at-most-one is assumed to be satisfied..
        return TRUE_var;
    else if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    { // we need to create a new variable..
        const var amo = new_var();
        bool nc = at_most_one(c_lits, amo);
        assert(nc);
        return amo;
    }
}

var sat_core::new_exct_one(const std::vector<lit> &ls)
{
    assert(root_level());
    // we try to avoid creating a new variable..
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    lit p;
    size_t j = 0;
    std::string s_expr = "^";
    for (std::vector<lit>::const_iterator it0 = c_lits.begin(); it0 != c_lits.end(); ++it0)
        if (value(*it0) == True)
            for (std::vector<lit>::const_iterator it1 = it0 + 1; it1 != c_lits.end(); ++it1)
            {
                if (value(*it1) == True || *it1 == !p)
                    return FALSE_var; // the exact-one cannot be satisfied..
                else if (value(*it1) != False && *it1 != p)
                { // we need to include this literal in the exact-one..
                    p = *it1;
                    s_expr += to_string(p);
                    c_lits[j++] = p;
                }
            }
        else if (value(*it0) != False && *it0 != p)
        { // we need to include this literal in the exact-one..
            p = *it0;
            s_expr += to_string(p);
            c_lits[j++] = p;
        }
    c_lits.resize(j);

    if (c_lits.empty()) // an empty exact-one is assumed to be unsatisfable..
        return FALSE_var;
    else if (c_lits.size() == 1 && c_lits[0].get_sign())
        return c_lits[0].get_var();
    else if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    { // we need to create a new variable..
        const var eo = new_var();
        bool nc = exct_one(c_lits, eo);
        assert(nc);
        return eo;
    }
}

bool sat_core::eq(const lit &left, const lit &right, const var &p)
{
    assert(root_level());
    switch (value(p))
    {
    case True:
        switch (value(left))
        {
        case True:
            switch (value(right))
            {
            case True:
                return true; // the variables assume the same value..
            case False:
                return false; // the variables do not assume the same value..
            case Undefined:
                return enqueue(right); // the eq constraint is unique under the current assignment..
            }
        case False:
            switch (value(right))
            {
            case True:
                return false; // the variables do not assume the same value..
            case False:
                return true; // the variables assume the same value..
            case Undefined:
                return enqueue(!right); // the eq constraint is unique under the current assignment..
            }
        case Undefined:
            switch (value(right))
            {
            case True:
                return enqueue(left); // the eq constraint is unique under the current assignment..
            case False:
                return enqueue(!left); // the eq constraint is unique under the current assignment..
            case Undefined:
                break;
            }
        }
        break;
    case False:
        switch (value(left))
        {
        case True:
            switch (value(right))
            {
            case True:
                return false; // the variables do not assume the same value..
            case False:
                return true; // the variables assume the same value..
            case Undefined:
                return enqueue(!right); // the eq constraint is unique under the current assignment..
            }
        case False:
            switch (value(right))
            {
            case True:
                return true; // the variables assume the same value..
            case False:
                return false; // the variables do not assume the same value..
            case Undefined:
                return enqueue(right); // the eq constraint is unique under the current assignment..
            }
        case Undefined:
            switch (value(right))
            {
            case True:
                return enqueue(!left); // the eq constraint is unique under the current assignment..
            case False:
                return enqueue(left); // the eq constraint is unique under the current assignment..
            case Undefined:
                break;
            }
        }
        break;
    case Undefined:
        switch (value(left))
        {
        case True:
            switch (value(right))
            {
            case True:
                return enqueue(lit(p, false)); // the eq constraint is unique under the current assignment..
            case False:
                return enqueue(p); // the eq constraint is unique under the current assignment..
            case Undefined:
                break;
            }
        case False:
            switch (value(right))
            {
            case True:
                return enqueue(lit(p, false)); // the eq constraint is unique under the current assignment..
            case False:
                return enqueue(p); // the eq constraint is unique under the current assignment..
            case Undefined:
                break;
            }
        case Undefined:
            break;
        }
        break;
    }
    const std::string s_expr = "=" + (left.get_var() < right.get_var()) ? (to_string(left) + to_string(right)) : (to_string(right) + to_string(left));
    if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return eq(p, at_expr->second);
    else
    {
        if (!new_clause({lit(p, false), !left, right}))
            return false;
        if (!new_clause({lit(p, false), left, !right}))
            return false;
        if (!new_clause({p, !left, !right}))
            return false;
        exprs.emplace(s_expr, p);
        return true;
    }
}

bool sat_core::conj(const std::vector<lit> &ls, const var &p)
{
    assert(root_level());
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    lit c_p;
    size_t j = 0;
    std::string s_expr = "&";
    for (std::vector<lit>::const_iterator it = c_lits.begin(); it != c_lits.end(); ++it)
        if (value(*it) == False || *it == !c_p)
            return eq(p, FALSE_var); // the conjunction cannot be satisfied..
        else if (value(*it) != True && *it != c_p)
        { // we need to include this literal in the conjunction..
            c_p = *it;
            s_expr += to_string(c_p);
            c_lits[j++] = c_p;
        }
    c_lits.resize(j);

    if (c_lits.empty()) // an empty conjunction is assumed to be satisfied..
        return eq(p, TRUE_var);
    else if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return eq(p, at_expr->second);
    else
    {
        std::vector<lit> lits;
        lits.reserve(c_lits.size() + 1);
        lits.push_back(p);
        for (const auto &l : c_lits)
        {
            if (!new_clause({lit(p, false), l}))
                return false;
            lits.push_back(!l);
        }
        if (!new_clause(lits))
            return false;
        exprs.emplace(s_expr, p);
        return true;
    }
}

bool sat_core::disj(const std::vector<lit> &ls, const var &p)
{
    assert(root_level());
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    lit c_p;
    size_t j = 0;
    std::string s_expr = "|";
    for (std::vector<lit>::const_iterator it = c_lits.begin(); it != c_lits.end(); ++it)
        if (value(*it) == True || *it == !p)
            return TRUE_var; // the disjunction is already satisfied..
        else if (value(*it) != False && *it != c_p)
        { // we need to include this literal in the conjunction..
            c_p = *it;
            s_expr += to_string(c_p);
            c_lits[j++] = c_p;
        }
    c_lits.resize(j);

    if (c_lits.empty()) // an empty disjunction is assumed to be unsatisfable..
        return eq(p, FALSE_var);
    else if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return eq(p, at_expr->second);
    else
    {
        std::vector<lit> lits;
        lits.reserve(c_lits.size() + 1);
        lits.push_back(lit(p, false));
        for (const auto &l : c_lits)
        {
            if (!new_clause({!l, p}))
                return false;
            lits.push_back(l);
        }
        if (!new_clause(lits))
            return false;
        exprs.emplace(s_expr, p);
        return true;
    }
}

bool sat_core::at_most_one(const std::vector<lit> &ls, const var &p)
{
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    lit c_p;
    size_t j = 0;
    std::string s_expr = "amo";
    for (std::vector<lit>::const_iterator it0 = c_lits.begin(); it0 != c_lits.end(); ++it0)
        if (value(*it0) == True)
            for (std::vector<lit>::const_iterator it1 = it0 + 1; it1 != c_lits.end(); ++it1)
            {
                if (value(*it1) == True || *it1 == !c_p)
                    return FALSE_var; // the at-most-one cannot be satisfied..
                else if (value(*it1) != False && *it1 != c_p)
                { // we need to include this literal in the exact-one..
                    c_p = *it1;
                    s_expr += to_string(c_p);
                    c_lits[j++] = c_p;
                }
            }
        else if (value(*it0) != False && *it0 != c_p)
        { // we need to include this literal in the at-most-one..
            c_p = *it0;
            s_expr += to_string(c_p);
            c_lits[j++] = c_p;
        }
    c_lits.resize(j);

    if (c_lits.empty() || c_lits.size() == 1) // an empty or a singleton at-most-one is assumed to be satisfied..
        return TRUE_var;
    else if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return eq(p, at_expr->second);
    else if (ls.size() < 5)
    { // we use the standard encoding..
        for (size_t i = 0; i < ls.size(); ++i)
            for (size_t j = i + 1; j < ls.size(); ++j)
                if (!new_clause({!ls[i], !ls[j], lit(p, false)}))
                    return false;
        exprs.emplace(s_expr, p);
        return true;
    }
    else
    {
        double ps = ceil(sqrt(ls.size()));
        double qs = ceil(ls.size() / ps);

        std::vector<lit> u, v;
        for (size_t i = 0; i < ps; ++i)
            u.push_back(new_var());
        for (size_t j = 0; j < qs; ++j)
            v.push_back(new_var());

        if (!at_most_one(u, p) || !at_most_one(v, p))
            return false;

        for (size_t i = 0; i < ps; ++i)
            for (size_t j = 0; j < qs; ++j)
                if (size_t k = static_cast<size_t>(i * qs + j); k < ls.size())
                    if (!new_clause({!ls[k], u[i], lit(p, false)}) || !new_clause({!ls[k], v[j], lit(p, false)}))
                        return false;

        exprs.emplace(s_expr, p);
        return true;
    }
}

bool sat_core::exct_one(const std::vector<lit> &ls, const var &p)
{
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    lit c_p;
    size_t j = 0;
    std::string s_expr = "^";
    for (std::vector<lit>::const_iterator it0 = c_lits.begin(); it0 != c_lits.end(); ++it0)
        if (value(*it0) == True)
            for (std::vector<lit>::const_iterator it1 = it0 + 1; it1 != c_lits.end(); ++it1)
            {
                if (value(*it1) == True || *it1 == !c_p)
                    return FALSE_var; // the exact-one cannot be satisfied..
                else if (value(*it1) != False && *it1 != c_p)
                { // we need to include this literal in the exact-one..
                    c_p = *it1;
                    s_expr += to_string(c_p);
                    c_lits[j++] = c_p;
                }
            }
        else if (value(*it0) != False && *it0 != c_p)
        { // we need to include this literal in the exact-one..
            c_p = *it0;
            s_expr += to_string(c_p);
            c_lits[j++] = c_p;
        }
    c_lits.resize(j);

    if (c_lits.empty()) // an empty exact-one is assumed to be unsatisfable..
        return FALSE_var;
    else if (c_lits.size() == 1 && c_lits[0].get_sign())
        return c_lits[0].get_var();
    else if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    {
        if (!disj(ls, p) || !at_most_one(ls, p))
            return false;
        exprs.emplace(s_expr, p);
        return true;
    }
}

bool sat_core::assume(const lit &p)
{
    trail_lim.push_back(trail.size());
    for (const auto &th : theories)
        th->push();
    return enqueue(p);
}

void sat_core::pop()
{
    while (trail_lim.back() < trail.size())
        pop_one();
    trail_lim.pop_back();

    for (const auto &th : theories)
        th->pop();
}

bool sat_core::simplify_db()
{
    assert(root_level());
    std::vector<lit> cnfl;
    if (!propagate(cnfl))
        return false;
    size_t j = 0;
    for (size_t i = 0; i < constrs.size(); ++i)
        if (constrs[i]->simplify())
            constrs[i]->remove();
        else
            constrs[j++] = constrs[i];
    constrs.resize(j);
    return true;
}

bool sat_core::check()
{
    std::vector<lit> cnfl;
    while (true)
        if (!propagate(cnfl))
        {
            if (root_level())
                return false;
            std::vector<lit> no_good;
            size_t bt_level;
            // we analyze the conflict..
            analyze(cnfl, no_good, bt_level);
            assert(cnfl.empty());
            while (decision_level() > bt_level)
                pop();
            // we record the no-good..
            record(no_good);
        }
        else
            return true;
}

bool sat_core::check(const std::vector<lit> &lits)
{
    const size_t c_rl = decision_level(); // the current root-level..
    size_t c_dl;                          // the current decision-level..
    for (const auto &p : lits)
    {
        c_dl = decision_level();
        if (!assume(p) || !check() || decision_level() <= c_dl)
        {
            while (decision_level() > c_rl)
                pop();
            return false;
        }
    }
    assert(c_rl + lits.size() == decision_level());
    while (decision_level() > c_rl)
        pop();
    return true;
}

bool sat_core::propagate(std::vector<lit> &cnfl)
{
    while (!prop_q.empty())
    {
        // we propagate sat constraints..
        std::vector<clause *> tmp;
        std::swap(tmp, watches[index(prop_q.front())]);
        for (size_t i = 0; i < tmp.size(); ++i)
        {
            if (!tmp[i]->propagate(prop_q.front()))
            {
                // constraint is conflicting..
                for (size_t j = i + 1; j < tmp.size(); ++j)
                    watches[index(prop_q.front())].push_back(tmp[j]);
                assert(std::count_if(tmp[i]->lits.begin(), tmp[i]->lits.end(), [&](const lit &p) { return std::find(watches[index(!p)].begin(), watches[index(!p)].end(), tmp[i]) != watches[index(!p)].end(); }) == 2);
                while (!prop_q.empty())
                    prop_q.pop();
                cnfl.insert(cnfl.begin(), tmp[i]->lits.begin(), tmp[i]->lits.end());
                return false;
            }
            assert(std::count_if(tmp[i]->lits.begin(), tmp[i]->lits.end(), [&](const lit &p) { return std::find(watches[index(!p)].begin(), watches[index(!p)].end(), tmp[i]) != watches[index(!p)].end(); }) == 2);
        }

        // we perform theory propagation..
        if (const auto bnds_it = bounds.find(prop_q.front().get_var()); bnds_it != bounds.end())
        {
            for (const auto &th : bnds_it->second)
                if (!th->propagate(prop_q.front(), cnfl))
                {
                    assert(!cnfl.empty());
                    while (!prop_q.empty())
                        prop_q.pop();
                    return false;
                }
            if (root_level()) // since this variable will no more be assigned, we can perform some cleanings..
                bounds.erase(bnds_it);
        }

        prop_q.pop();
    }

    // we check theories..
    for (const auto &th : theories)
        if (!th->check(cnfl))
        {
            assert(!cnfl.empty());
            return false;
        }

    return true;
}

void sat_core::analyze(std::vector<lit> &cnfl, std::vector<lit> &out_learnt, size_t &out_btlevel)
{
    assert(std::all_of(cnfl.begin(), cnfl.end(), [this](const lit &lt) { return value(lt) != Undefined; })); // all these literals must have been assigned for being a conflict..
    std::set<var> seen;
    int counter = 0; // this is the number of variables of the current decision level that have already been seen..
    lit p;
    std::vector<lit> p_reason;
    std::swap(p_reason, cnfl); // cnfl is now empty..
    out_learnt.push_back(p);
    out_btlevel = 0;
    do
    {
        // trace reason for 'p'..
        for (const auto &q : p_reason) // the order in which these literals are visited is not relevant..
            if (seen.insert(q.get_var()).second)
            {
                assert(value(q) == False); // this literal should have propagated the clause..
                if (level[q.get_var()] == decision_level())
                    counter++;
                else if (level[q.get_var()] > 0) // exclude variables from decision level 0..
                {
                    out_learnt.push_back(q); // this literal has been assigned in a previous decision level..
                    out_btlevel = std::max(out_btlevel, level[q.get_var()]);
                }
            }
        // select next literal to look at..
        do
        {
            p = trail.back();
            assert(level[p.get_var()] == decision_level()); // this variable must have been assigned at the current decision level..
            if (reason[p.get_var()])                        // 'p' can be the asserting literal..
            {
                assert(reason[p.get_var()]->lits[0] == p);                                                                                                         // a consequence of propagating the clause is the assignment of literal 'p'..
                assert(value(p) == True);                                                                                                                          // 'p' has been propagated as true..
                assert(std::all_of(reason[p.get_var()]->lits.begin() + 1, reason[p.get_var()]->lits.end(), [this](const lit &lt) { return value(lt) == False; })); // all these literals must have been assigned as false for propagating 'p'..
                p_reason.clear();
                p_reason.insert(p_reason.end(), reason[p.get_var()]->lits.begin() + 1, reason[p.get_var()]->lits.end());
            }
            pop_one();
        } while (!seen.count(p.get_var()));
        counter--;
    } while (counter > 0);
    // 'p' is now the first Unique Implication Point (UIP), possibly the asserting literal, that led to the conflict..
    assert(value(p) == Undefined);
    assert(std::all_of(out_learnt.begin() + 1, out_learnt.end(), [this](const lit &lt) { return value(lt) == False; })); // all these literals must have been assigned as false for propagating 'p'..
    out_learnt[0] = !p;
}

void sat_core::record(const std::vector<lit> &lits)
{
    assert(value(lits[0]) == Undefined);
    assert(std::count_if(lits.begin(), lits.end(), [this](const lit &p) { return value(p) == True; }) == 0);
    assert(std::count_if(lits.begin(), lits.end(), [this](const lit &p) { return value(p) == Undefined; }) == 1);
    assert(std::count_if(lits.begin(), lits.end(), [this](const lit &p) { return value(p) == False; }) == lits.size() - 1);
    if (lits.size() == 1)
    {
        assert(root_level());
        bool e = enqueue(lits[0]);
        assert(e);
    }
    else
    {
        std::vector<lit> c_lits = lits;
        // we sort literals according to descending order of variable assignment (except for the first literal which is now unassigned)..
        std::sort(c_lits.begin() + 1, c_lits.end(), [this](const lit &a, const lit &b) { return level[a.get_var()] > level[b.get_var()]; });
        clause *c = new clause(*this, c_lits);
        bool e = enqueue(c_lits[0], c);
        assert(e);
        constrs.push_back(c);
    }
}

bool sat_core::enqueue(const lit &p, clause *const c)
{
    switch (value(p))
    {
    case True:
        return true;
    case False:
        return false;
    case Undefined:
    {
        assigns[p.get_var()] = p.get_sign() ? True : False;
        level[p.get_var()] = decision_level();
        reason[p.get_var()] = c;
        trail.push_back(p);
        prop_q.push(p);
        // we notify the listeners that a listening variable has been assigned..
        if (const auto at_p = listening.find(p.get_var()); at_p != listening.end())
        {
            for (const auto &l : at_p->second)
                l->sat_value_change(p.get_var());
            if (root_level()) // since this variable will no more be assigned, we can perform some cleanings..
                listening.erase(at_p);
        }
        return true;
    }
    default:
        assert(0);
        return false;
    }
}

void sat_core::pop_one()
{
    const var v = trail.back().get_var();
    assigns[v] = Undefined;
    level[v] = 0;
    reason[v] = nullptr;
    trail.pop_back();
    if (const auto at_v = listening.find(v); at_v != listening.end())
        for (const auto &l : at_v->second)
            l->sat_value_change(v);
}
} // namespace smt
