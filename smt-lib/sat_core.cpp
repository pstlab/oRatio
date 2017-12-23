#include "sat_core.h"
#include "clause.h"
#include "theory.h"
#include "sat_value_listener.h"
#include <algorithm>
#include <cassert>

namespace smt
{

sat_core::sat_core()
{
    var c_false = new_var();
    var c_true = new_var();
    assert(c_false == FALSE_var);
    assert(c_true == TRUE_var);
    assigns[FALSE_var] = False;
    assigns[TRUE_var] = True;
    level[FALSE_var] = 0;
    level[TRUE_var] = 0;
}

sat_core::~sat_core()
{
    for (const auto &c : constrs)
        delete c;
}

const var sat_core::new_var()
{
    const var id = assigns.size();
    watches.push_back(std::vector<clause *>());
    watches.push_back(std::vector<clause *>());
    assigns.push_back(Undefined);
    exprs.insert({"b" + std::to_string(id), id});
    level.push_back(0);
    reason.push_back(nullptr);
    return id;
}

bool sat_core::new_clause(const std::vector<lit> &lits)
{
    assert(root_level());

    std::vector<lit> c_lits;
    for (const auto &l : lits)
        switch (value(l))
        {
        case True:
            return true; // the clause is already satisfied..
        case Undefined:
            bool found = false;
            for (const auto &c_l : c_lits)
                if (c_l == l)
                {
                    found = true;
                    break;
                }
                else if (c_l == !l)
                    return true; // the clause represents a tautology..
            if (!found)
                c_lits.push_back(l);
        }

    switch (c_lits.size())
    {
    case 0:
        return false;
    case 1:
        return enqueue(c_lits.at(0));
    default:
        constrs.push_back(new clause(*this, c_lits));
        return true;
    }
}

const var sat_core::new_eq(const lit &left, const lit &right)
{
    assert(root_level());
    if (left == right)
        return TRUE_var;
    if (left.v > right.v)
        return new_eq(right, left);
    const std::string s_expr = (left.sign ? "b" : "!b") + std::to_string(left.v) + " == " + (right.sign ? "b" : "!b") + std::to_string(right.v);
    const auto at_expr = exprs.find(s_expr);
    if (at_expr != exprs.end()) // the expression already exists..
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
        exprs.insert({s_expr, e});
        return e;
    }
}

const var sat_core::new_conj(const std::vector<lit> &ls)
{
    assert(root_level());
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.v > l1.v; });
    std::string s_expr;
    for (std::vector<lit>::const_iterator it = c_lits.begin(); it != c_lits.end(); ++it)
    {
        if (it != c_lits.begin())
            s_expr += " & ";
        s_expr += (it->sign ? "b" : "!b") + std::to_string(it->v);
    }
    const auto at_expr = exprs.find(s_expr);
    if (at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    {
        // we need to create a new variable..
        const var c = new_var();
        std::vector<lit> lits;
        lits.push_back(c);
        bool nc;
        for (const auto &l : ls)
        {
            nc = new_clause({lit(c, false), l});
            assert(nc);
            lits.push_back(!l);
        }
        nc = new_clause(lits);
        assert(nc);
        exprs.insert({s_expr, c});
        return c;
    }
}

const var sat_core::new_disj(const std::vector<lit> &ls)
{
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.v > l1.v; });
    std::string s_expr;
    for (std::vector<lit>::const_iterator it = c_lits.begin(); it != c_lits.end(); ++it)
    {
        if (it != c_lits.begin())
            s_expr += " | ";
        s_expr += (it->sign ? "b" : "!b") + std::to_string(it->v);
    }
    const auto at_expr = exprs.find(s_expr);
    if (at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    {
        // we need to create a new variable..
        const var d = new_var();
        std::vector<lit> lits;
        lits.push_back(lit(d, false));
        bool nc;
        for (const auto &l : ls)
        {
            nc = new_clause({!l, d});
            assert(nc);
            lits.push_back(l);
        }
        nc = new_clause(lits);
        assert(nc);
        exprs.insert({s_expr, d});
        return d;
    }
}

const var sat_core::new_exct_one(const std::vector<lit> &ls)
{
    std::vector<lit> c_lits = ls;
    std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return l0.v > l1.v; });
    std::string s_expr;
    for (std::vector<lit>::const_iterator it = c_lits.begin(); it != c_lits.end(); ++it)
    {
        if (it != c_lits.begin())
            s_expr += " ^ ";
        s_expr += (it->sign ? "b" : "!b") + std::to_string(it->v);
    }
    const auto at_expr = exprs.find(s_expr);
    if (at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else
    {
        // we need to create a new variable..
        const var eo = new_var();
        std::vector<lit> lits;
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
        exprs.insert({s_expr, eo});
        return eo;
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
            cnfl.clear();
        }
        else
            return true;
    }
}

bool sat_core::check(const std::vector<lit> &lits)
{
    size_t c_level = decision_level();
    std::vector<lit> cnfl;
    for (const auto &p : lits)
    {
        // notice that these literals can be modified by propagation..
        if (!assume(p) || !propagate(cnfl))
        {
            while (decision_level() > c_level)
                pop();
            return false;
        }
    }
    while (decision_level() > c_level)
        pop();
    return true;
}

bool sat_core::propagate(std::vector<lit> &cnfl)
{
    while (!prop_q.empty())
    {
        // we propagate sat constraints..
        std::vector<clause *> tmp = std::move(watches.at(index(prop_q.front())));
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
        const auto bnds_it = bounds.find(prop_q.front().v);
        if (bnds_it != bounds.end())
            for (const auto &th : bnds_it->second)
                if (!th->propagate(prop_q.front(), cnfl))
                {
                    assert(!cnfl.empty());
                    while (!prop_q.empty())
                        prop_q.pop();
                    return false;
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

void sat_core::analyze(const std::vector<lit> &cnfl, std::vector<lit> &out_learnt, size_t &out_btlevel)
{
    assert(std::all_of(cnfl.begin(), cnfl.end(), [&](const lit &lt) { return value(lt) != Undefined; })); // all these literals must have been assigned for being a conflict..
    std::set<var> seen;
    int counter = 0; // this is the number of variables of the current decision level that have already been seen..
    lit p;
    std::vector<lit> p_reason = std::move(cnfl);
    out_learnt.push_back(lit());
    out_btlevel = 0;
    do
    {
        // trace reason for 'p'..
        for (const auto &q : p_reason) // the order in which these literals are visited is not relevant..
            if (seen.find(q.v) == seen.end())
            {
                assert(value(q) == False); // this literal should have propagated the clause..
                seen.insert(q.v);
                if (level.at(q.v) == decision_level())
                    counter++;
                else if (level.at(q.v) > 0) // exclude variables from decision level 0..
                {
                    out_learnt.push_back(q); // this literal has been assigned in a previous decision level..
                    out_btlevel = std::max(out_btlevel, level.at(q.v));
                }
            }
        // select next literal to look at..
        do
        {
            p = trail.back();
            assert(level.at(p.v) == decision_level()); // this variable must have been assigned at the current decision level..
            if (reason.at(p.v))                        // 'p' can be the asserting literal..
            {
                assert(reason.at(p.v)->lits.at(0) == p);                                                                                              // a consequence of propagating the clause is the assignment of literal 'p'..
                assert(value(p) == True);                                                                                                             // 'p' has been propagated as true..
                assert(std::all_of(reason.at(p.v)->lits.begin() + 1, reason.at(p.v)->lits.end(), [&](const lit &lt) { return value(lt) == False; })); // all these literals must have been assigned as false for propagating 'p'..
                p_reason.clear();
                p_reason.insert(p_reason.end(), reason.at(p.v)->lits.begin() + 1, reason.at(p.v)->lits.end());
            }
            pop_one();
        } while (seen.find(p.v) == seen.end());
        counter--;
    } while (counter > 0);
    // 'p' is now the first Unique Implication Point (UIP), possibly the asserting literal, that led to the conflict..
    assert(value(p) == Undefined);
    assert(std::all_of(out_learnt.begin() + 1, out_learnt.end(), [&](const lit &lt) { return value(lt) == False; })); // all these literals must have been assigned as false for propagating 'p'..
    out_learnt.insert(out_learnt.begin(), !p);
}

void sat_core::record(const std::vector<lit> &lits)
{
    assert(value(lits.at(0)) == Undefined);
    assert(std::count_if(lits.begin(), lits.end(), [&](const lit &p) { return value(p) == True; }) == 0);
    assert(std::count_if(lits.begin(), lits.end(), [&](const lit &p) { return value(p) == Undefined; }) == 1);
    assert(std::count_if(lits.begin(), lits.end(), [&](const lit &p) { return value(p) == False; }) == lits.size() - 1);
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
        std::sort(c_lits.begin() + 1, c_lits.end(), [&](lit &a, lit &b) { return level.at(a.v) > level.at(b.v); });
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
        assigns[p.v] = p.sign ? True : False;
        level[p.v] = decision_level();
        reason[p.v] = c;
        trail.push_back(p);
        prop_q.push(p);
        const auto at_p = listening.find(p.v);
        if (at_p != listening.end())
            for (const auto &l : at_p->second)
                l->sat_value_change(p.v);
        return true;
    }
    default:
        std::unexpected();
    }
}

void sat_core::pop_one()
{
    const var v = trail.back().v;
    assigns[v] = Undefined;
    level[v] = 0;
    reason[v] = nullptr;
    trail.pop_back();
    const auto at_v = listening.find(v);
    if (at_v != listening.end())
        for (const auto &l : at_v->second)
            l->sat_value_change(v);
}
}