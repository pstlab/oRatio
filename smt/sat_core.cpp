#include "sat_core.h"
#include "clause.h"
#include "sat_value_listener.h"
#include <algorithm>
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

    // we filter out false literals..
    std::vector<lit> c_lits;
    c_lits.reserve(lits.size());
    for (const auto &l : lits)
        switch (value(l))
        {
        case True:
            return true; // the clause is already satisfied..
        case False:
            break; // we skip false literals..
        case Undefined:
            if (std::any_of(c_lits.begin(), c_lits.end(), [this, l](const auto &c_l) { return c_l == !l; }))
                return true; // the clause represents a tautology..
            else if (std::none_of(c_lits.begin(), c_lits.end(), [this, l](const auto &c_l) { return c_l == l; }))
                c_lits.push_back(l); // we skip duplicates..
        }

    switch (c_lits.size())
    {
    case 0: // the clause is unsatisfable..
        return false;
    case 1: // the clause is unique under the current assignment..
        return enqueue(c_lits.at(0));
    default:
        constrs.push_back(new clause(*this, c_lits));
        return true;
    }
}

var sat_core::new_eq(const lit &left, const lit &right)
{
    assert(root_level());
    if (left == right)
        return TRUE_var;
    if (left.get_var() > right.get_var())
        return new_eq(right, left);
    const std::string s_expr = (left.get_sign() ? "b" : "!b") + std::to_string(left.get_var()) + " == " + (right.get_sign() ? "b" : "!b") + std::to_string(right.get_var());
    if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    {
        // we need to create a new variable..
        const var e = new_var();
        bool nc;
        nc = new_clause({lit(e, false), !left, right});
        assert(nc);
        nc = new_clause({lit(e, false), left, !right});
        assert(nc);
        nc = new_clause({e, !left, !right});
        assert(nc);
        exprs.emplace(s_expr, e);
        return e;
    }
}

var sat_core::new_conj(const std::vector<lit> &ls)
{
    assert(root_level());
    std::vector<lit> c_lits;
    c_lits.reserve(ls.size());
    for (const auto &l : ls)
        switch (value(l))
        {
        case True:
            break; // we skip true literals..
        case False:
            return FALSE_var; // the conjunction cannot be satisfied..
        case Undefined:
            if (std::any_of(c_lits.begin(), c_lits.end(), [this, l](const auto &c_l) { return c_l == !l; }))
                return FALSE_var; // the conjunction cannot be satisfied..
            else
                c_lits.push_back(l);
        }

    if (c_lits.empty()) // an empty conjunction is assumed to be satisfied..
        return TRUE_var;

    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    std::string s_expr;
    for (std::vector<lit>::const_iterator it = c_lits.cbegin(); it != c_lits.cend(); ++it)
    {
        if (it != c_lits.cbegin())
            s_expr += " & ";
        s_expr += (it->get_sign() ? "b" : "!b") + std::to_string(it->get_var());
    }
    if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    {
        // we need to create a new variable..
        const var c = new_var();
        std::vector<lit> lits;
        lits.reserve(c_lits.size() + 1);
        lits.push_back(c);
        bool nc;
        for (const auto &l : c_lits)
        {
            nc = new_clause({lit(c, false), l});
            assert(nc);
            lits.push_back(!l);
        }
        nc = new_clause(lits);
        assert(nc);
        exprs.emplace(s_expr, c);
        return c;
    }
}

var sat_core::new_disj(const std::vector<lit> &ls)
{
    assert(root_level());
    std::vector<lit> c_lits;
    c_lits.reserve(ls.size());
    for (const auto &l : ls)
        switch (value(l))
        {
        case True:
            return TRUE_var; // the disjunction is already satisfied..
        case False:
            break; // we skip false literals..
        case Undefined:
            if (std::any_of(c_lits.begin(), c_lits.end(), [this, l](const auto &c_l) { return c_l == !l; }))
                return TRUE_var; // the disjunction is already satisfied..
            else
                c_lits.push_back(l);
        }

    if (c_lits.empty()) // an empty disjunction is assumed to be unsatisfable..
        return FALSE_var;

    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    std::string s_expr;
    for (std::vector<lit>::const_iterator it = c_lits.cbegin(); it != c_lits.cend(); ++it)
    {
        if (it != c_lits.cbegin())
            s_expr += " | ";
        s_expr += (it->get_sign() ? "b" : "!b") + std::to_string(it->get_var());
    }
    if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    {
        // we need to create a new variable..
        const var d = new_var();
        std::vector<lit> lits;
        lits.reserve(c_lits.size() + 1);
        lits.push_back(lit(d, false));
        bool nc;
        for (const auto &l : c_lits)
        {
            nc = new_clause({!l, d});
            assert(nc);
            lits.push_back(l);
        }
        nc = new_clause(lits);
        assert(nc);
        exprs.emplace(s_expr, d);
        return d;
    }
}

var sat_core::new_exct_one(const std::vector<lit> &ls)
{
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    std::string s_expr;
    for (std::vector<lit>::const_iterator it = c_lits.cbegin(); it != c_lits.cend(); ++it)
    {
        if (it != c_lits.cbegin())
            s_expr += " ^ ";
        s_expr += (it->get_sign() ? "b" : "!b") + std::to_string(it->get_var());
    }
    if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    {
        // we need to create a new variable..
        const var eo = new_var();
        std::vector<lit> lits;
        lits.reserve(ls.size() + 1);
        lits.push_back(lit(eo, false));
        bool nc;
        for (size_t i = 0; i < ls.size(); i++)
        {
            for (size_t j = i + 1; j < ls.size(); j++)
            {
                nc = new_clause({!ls.at(i), !ls.at(j), lit(eo, false)});
                assert(nc);
            }
            lits.push_back(ls.at(i));
        }
        nc = new_clause(lits);
        assert(nc);
        exprs.emplace(s_expr, eo);
        return eo;
    }
}

bool sat_core::eq(const lit &left, const lit &right, const var &p)
{
    assert(root_level());
    if (left == right)
        return true;
    if (left.get_var() > right.get_var())
        return eq(right, left);
    const std::string s_expr = (left.get_sign() ? "b" : "!b") + std::to_string(left.get_var()) + " == " + (right.get_sign() ? "b" : "!b") + std::to_string(right.get_var());
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
    std::vector<lit> c_lits;
    c_lits.reserve(ls.size());
    for (const auto &l : ls)
        switch (value(l))
        {
        case True:
            break; // we skip true literals..
        case False:
            return FALSE_var; // the conjunction cannot be satisfied..
        case Undefined:
            if (std::any_of(c_lits.begin(), c_lits.end(), [this, l](const auto &c_l) { return c_l == !l; }))
                return FALSE_var; // the conjunction cannot be satisfied..
            else
                c_lits.push_back(l);
        }

    if (c_lits.empty()) // an empty conjunction is assumed to be satisfied..
        return value(p) == True;

    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    std::string s_expr;
    for (std::vector<lit>::const_iterator it = c_lits.cbegin(); it != c_lits.cend(); ++it)
    {
        if (it != c_lits.cbegin())
            s_expr += " & ";
        s_expr += (it->get_sign() ? "b" : "!b") + std::to_string(it->get_var());
    }
    if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
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
    std::vector<lit> c_lits;
    c_lits.reserve(ls.size());
    for (const auto &l : ls)
        switch (value(l))
        {
        case True:
            return true; // the disjunction is already satisfied..
        case False:
            break; // we skip false literals..
        case Undefined:
            if (std::any_of(c_lits.begin(), c_lits.end(), [this, l](const auto &c_l) { return c_l == !l; }))
                return true; // the disjunction is already satisfied..
            else
                c_lits.push_back(l);
        }

    if (c_lits.empty()) // an empty disjunction is assumed to be unsatisfable..
        return value(p) == False;

    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    std::string s_expr;
    for (std::vector<lit>::const_iterator it = c_lits.cbegin(); it != c_lits.cend(); ++it)
    {
        if (it != c_lits.cbegin())
            s_expr += " | ";
        s_expr += (it->get_sign() ? "b" : "!b") + std::to_string(it->get_var());
    }
    if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
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

bool sat_core::exct_one(const std::vector<lit> &ls, const var &p)
{
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.get_var() < l1.get_var(); });
    std::string s_expr;
    for (std::vector<lit>::const_iterator it = c_lits.cbegin(); it != c_lits.cend(); ++it)
    {
        if (it != c_lits.cbegin())
            s_expr += " ^ ";
        s_expr += (it->get_sign() ? "b" : "!b") + std::to_string(it->get_var());
    }
    if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
        return eq(p, at_expr->second);
    else
    {
        std::vector<lit> lits;
        lits.reserve(ls.size() + 1);
        lits.push_back(lit(p, false));
        for (size_t i = 0; i < ls.size(); i++)
        {
            for (size_t j = i + 1; j < ls.size(); j++)
                if (!new_clause({!ls.at(i), !ls.at(j), lit(p, false)}))
                    return false;
            lits.push_back(ls.at(i));
        }
        if (!new_clause(lits))
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

bool sat_core::check()
{
    std::vector<lit> cnfl;
    while (true)
    {
        if (!propagate(cnfl))
        {
            if (root_level())
                return false;
            std::vector<lit> no_good;
            size_t bt_level;
            // we analyze the conflict..
            analyze(cnfl, no_good, bt_level);
            while (decision_level() > bt_level)
                pop();
            // we record the no-good..
            record(no_good);
        }
        else
        {
            if (root_level())
            { // we perform some cleanings..
                size_t j = 0;
                for (size_t i = 0; i < constrs.size(); ++i)
                    if (constrs.at(i)->simplify())
                        constrs.at(i)->remove();
                    else
                        constrs[j++] = constrs.at(i);
                constrs.resize(j);
            }
            return true;
        }
    }
}

bool sat_core::check(const std::vector<lit> &lits)
{
    trail_lim.push_back(trail.size());
    for (const auto &p : lits)
        if (!enqueue(p))
        {
            while (trail_lim.back() < trail.size())
                pop_one();
            trail_lim.pop_back();
            return false;
        }
    for (const auto &th : theories)
        th->push();
    if (std::vector<lit> cnfl; !propagate(cnfl))
    {
        pop();
        return false;
    }
    pop();
    return true;
}

bool sat_core::propagate(std::vector<lit> &cnfl)
{
    while (!prop_q.empty())
    {
        // we propagate sat constraints..
        std::vector<clause *> tmp;
        std::swap(tmp, watches.at(index(prop_q.front())));
        for (size_t i = 0; i < tmp.size(); i++)
        {
            if (!tmp.at(i)->propagate(prop_q.front()))
            {
                // constraint is conflicting..
                for (size_t j = i + 1; j < tmp.size(); j++)
                    watches.at(index(prop_q.front())).push_back(tmp.at(j));
                assert(std::count_if(tmp.at(i)->lits.begin(), tmp.at(i)->lits.end(), [&](const lit &p) { return std::find(watches.at(index(!p)).begin(), watches.at(index(!p)).end(), tmp.at(i)) != watches.at(index(!p)).end(); }) == 2);
                while (!prop_q.empty())
                    prop_q.pop();
                cnfl.insert(cnfl.begin(), tmp.at(i)->lits.begin(), tmp.at(i)->lits.end());
                return false;
            }
            assert(std::count_if(tmp.at(i)->lits.begin(), tmp.at(i)->lits.end(), [&](const lit &p) { return std::find(watches.at(index(!p)).begin(), watches.at(index(!p)).end(), tmp.at(i)) != watches.at(index(!p)).end(); }) == 2);
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
                if (level.at(q.get_var()) == decision_level())
                    counter++;
                else if (level.at(q.get_var()) > 0) // exclude variables from decision level 0..
                {
                    out_learnt.push_back(q); // this literal has been assigned in a previous decision level..
                    out_btlevel = std::max(out_btlevel, level.at(q.get_var()));
                }
            }
        // select next literal to look at..
        do
        {
            p = trail.back();
            assert(level.at(p.get_var()) == decision_level()); // this variable must have been assigned at the current decision level..
            if (reason.at(p.get_var()))                        // 'p' can be the asserting literal..
            {
                assert(reason.at(p.get_var())->lits.at(0) == p);                                                                                                         // a consequence of propagating the clause is the assignment of literal 'p'..
                assert(value(p) == True);                                                                                                                                // 'p' has been propagated as true..
                assert(std::all_of(reason.at(p.get_var())->lits.begin() + 1, reason.at(p.get_var())->lits.end(), [this](const lit &lt) { return value(lt) == False; })); // all these literals must have been assigned as false for propagating 'p'..
                p_reason.clear();
                p_reason.insert(p_reason.end(), reason.at(p.get_var())->lits.begin() + 1, reason.at(p.get_var())->lits.end());
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
    assert(value(lits.at(0)) == Undefined);
    assert(std::count_if(lits.begin(), lits.end(), [this](const lit &p) { return value(p) == True; }) == 0);
    assert(std::count_if(lits.begin(), lits.end(), [this](const lit &p) { return value(p) == Undefined; }) == 1);
    assert(std::count_if(lits.begin(), lits.end(), [this](const lit &p) { return value(p) == False; }) == lits.size() - 1);
    if (lits.size() == 1)
    {
        assert(root_level());
        bool e = enqueue(lits.at(0));
        assert(e);
    }
    else
    {
        std::vector<lit> c_lits(lits.begin(), lits.end());
        // we sort literals according to descending order of variable assignment (except for the first literal which is now unassigned)..
        std::sort(c_lits.begin() + 1, c_lits.end(), [this](lit &a, lit &b) { return level.at(a.get_var()) > level.at(b.get_var()); });
        clause *c = new clause(*this, c_lits);
        bool e = enqueue(c_lits.at(0), c);
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
