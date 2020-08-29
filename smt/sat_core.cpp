#include "sat_core.h"
#include "clause.h"
#include "sat_value_listener.h"
#include "theory.h"
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

    var sat_core::new_var() noexcept
    {
        const var id = assigns.size();
        watches.push_back(std::vector<constr *>());
        watches.push_back(std::vector<constr *>());
        assigns.push_back(Undefined);
        exprs.emplace("b" + std::to_string(id), id);
        level.push_back(0);
        reason.push_back(nullptr);
        return id;
    }

    bool sat_core::new_clause(const std::vector<lit> &lits) noexcept
    {
        assert(root_level());
        // we check if the clause is already satisfied and filter out false/duplicate literals..
        std::vector<lit> c_lits = lits;
        std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return variable(l0) < variable(l1); });
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

    lit sat_core::new_eq(const lit &left, const lit &right) noexcept
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
                return sign(left) == sign(right) ? right : !right;
            }
        case False:
            switch (value(right))
            {
            case True:
                return FALSE_var; // the variables cannot assume the same value..
            case False:
                return TRUE_var; // the variables assume the same value..
            case Undefined:
                return sign(left) == sign(right) ? !right : right;
            }
        case Undefined:
            switch (value(right))
            {
            case True:
                return sign(left) == sign(right) ? left : !left;
            case False:
                return sign(left) == sign(right) ? !left : left;
            case Undefined:
                break;
            }
        }

        const std::string s_expr = "=" + (left < right ? (to_string(left) + to_string(right)) : (to_string(right) + to_string(left)));
        if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
            return at_expr->second;
        else
        { // we need to create a new variable..
            const var ctr = new_var();
            if (!new_clause({lit(ctr, false), !left, right}))
                return FALSE_var;
            if (!new_clause({lit(ctr, false), left, !right}))
                return FALSE_var;
            if (!new_clause({ctr, !left, !right}))
                return FALSE_var;
            exprs.emplace(s_expr, ctr);
            return ctr;
        }
    }

    lit sat_core::new_conj(const std::vector<lit> &ls) noexcept
    {
        assert(root_level());
        // we try to avoid creating a new variable..
        std::vector<lit> c_lits = ls;
        std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return variable(l0) < variable(l1); });
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
        else if (c_lits.size() == 1)
            return c_lits[0];
        else if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
            return at_expr->second;
        else
        { // we need to create a new variable..
            const var ctr = new_var();
            std::vector<lit> lits;
            lits.reserve(c_lits.size() + 1);
            lits.push_back(ctr);
            for (const auto &l : c_lits)
            {
                if (!new_clause({lit(ctr, false), l}))
                    return FALSE_var;
                lits.push_back(!l);
            }
            if (!new_clause(lits))
                return FALSE_var;
            exprs.emplace(s_expr, ctr);
            return ctr;
        }
    }

    lit sat_core::new_disj(const std::vector<lit> &ls) noexcept
    {
        assert(root_level());
        // we try to avoid creating a new variable..
        std::vector<lit> c_lits = ls;
        std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return variable(l0) < variable(l1); });
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
        else if (c_lits.size() == 1)
            return c_lits[0];
        else if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
            return at_expr->second;
        else
        { // we need to create a new variable..
            const var ctr = new_var();
            std::vector<lit> lits;
            lits.reserve(c_lits.size() + 1);
            lits.push_back(lit(ctr, false));
            for (const auto &l : c_lits)
            {
                if (!new_clause({!l, ctr}))
                    return FALSE_var;
                lits.push_back(l);
            }
            if (!new_clause(lits))
                return FALSE_var;
            exprs.emplace(s_expr, ctr);
            return ctr;
        }
    }

    lit sat_core::new_at_most_one(const std::vector<lit> &ls) noexcept
    {
        assert(root_level());
        // we try to avoid creating a new variable..
        std::vector<lit> c_lits = ls;
        std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return variable(l0) < variable(l1); });
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
                    { // we need to include this literal in the at-most-one..
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
            if (ls.size() < 4)
            { // we use the standard encoding..
                const var ctr = new_var();
                for (size_t i = 0; i < ls.size(); ++i)
                    for (size_t j = i + 1; j < ls.size(); ++j)
                        if (!new_clause({!ls[i], !ls[j], lit(ctr, false)}))
                            return FALSE_var;
                exprs.emplace(s_expr, ctr);
                return ctr;
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

                const lit ctr = new_conj({new_at_most_one(u), new_at_most_one(v)});

                for (size_t i = 0; i < ps; ++i)
                    for (size_t j = 0; j < qs; ++j)
                        if (size_t k = static_cast<size_t>(i * qs + j); k < ls.size())
                            if (!new_clause({!ls[k], u[i], !ctr}) || !new_clause({!ls[k], v[j], !ctr}))
                                return FALSE_var;

                exprs.emplace(s_expr, ctr);
                return ctr;
            }
        }
    }

    lit sat_core::new_exct_one(const std::vector<lit> &ls) noexcept
    {
        assert(root_level());
        // we try to avoid creating a new variable..
        std::vector<lit> c_lits = ls;
        std::sort(c_lits.begin(), c_lits.end(), [](const lit &l0, const lit &l1) { return variable(l0) < variable(l1); });
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
        else if (c_lits.size() == 1 && sign(c_lits[0]))
            return c_lits[0];
        else if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.end()) // the expression already exists..
            return at_expr->second;
        else
        { // we need to create a new variable..
            const lit ctr = new_at_most_one(c_lits);
            c_lits.push_back(!ctr);
            if (!new_clause(c_lits))
                return FALSE_var;
            exprs.emplace(s_expr, ctr);
            return ctr;
        }
    }

    bool sat_core::assume(const lit &p) noexcept
    {
        assert(prop_q.empty());
        trail_lim.push_back(trail.size());
        for (const auto &th : theories)
            th->push();
        return enqueue(p) && propagate();
    }

    void sat_core::pop() noexcept
    {
        while (trail_lim.back() < trail.size())
            pop_one();
        trail_lim.pop_back();

        for (const auto &th : theories)
            th->pop();
    }

    bool sat_core::simplify_db() noexcept
    {
        assert(root_level());
        if (!propagate())
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

    bool sat_core::propagate() noexcept
    {
        lit p;
    main_loop:
        while (!prop_q.empty())
        { // we first propagate sat constraints..
            p = prop_q.front();
            prop_q.pop();
            std::vector<constr *> tmp;
            std::swap(tmp, watches[index(p)]);
            for (size_t i = 0; i < tmp.size(); ++i)
                if (!tmp[i]->propagate(p))
                { // the constraint is conflicting..
                    for (size_t j = i + 1; j < tmp.size(); ++j)
                        watches[index(p)].push_back(tmp[j]);
                    while (!prop_q.empty())
                        prop_q.pop();

                    if (root_level())
                        return false;
                    std::vector<lit> no_good;
                    size_t bt_level;
                    // we analyze the conflict..
                    analyze(*tmp[i], no_good, bt_level);
                    while (decision_level() > bt_level)
                        pop();
                    // we record the no-good..
                    record(no_good);

                    goto main_loop;
                }

            // we then perform theory propagation..
            if (const auto bnds_it = bounds.find(variable(p)); bnds_it != bounds.end())
            {
                for (const auto &th : bnds_it->second)
                    if (!th->propagate(p))
                    {
                        while (!prop_q.empty())
                            prop_q.pop();

                        if (root_level())
                            return false;
                        std::vector<lit> no_good;
                        size_t bt_level;
                        // we analyze the conflict..
                        constr *cnfl = new clause(*this, th->cnfl);
                        analyze(*cnfl, no_good, bt_level);
                        delete cnfl;
                        while (decision_level() > bt_level)
                            pop();
                        // we record the no-good..
                        record(no_good);

                        goto main_loop;
                    }
                if (root_level()) // since this variable will no more be assigned, we can perform some cleanings..
                    bounds.erase(bnds_it);
            }
        }

        // finally, we check theories..
        for (const auto &th : theories)
            if (!th->check())
            {
                if (root_level())
                    return false;
                std::vector<lit> no_good;
                size_t bt_level;
                // we analyze the conflict..
                constr *cnfl = new clause(*this, th->cnfl);
                analyze(*cnfl, no_good, bt_level);
                delete cnfl;
                while (decision_level() > bt_level)
                    pop();
                // we record the no-good..
                record(no_good);

                goto main_loop;
            }

        return true;
    }

    bool sat_core::check(const std::vector<lit> &lits) noexcept
    {
        const size_t c_rl = decision_level(); // the current root-level..
        size_t c_dl;                          // the current decision-level..
        for (const auto &p : lits)
        {
            c_dl = decision_level();
            if (!assume(p) || !propagate() || decision_level() <= c_dl)
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

    void sat_core::analyze(constr &cnfl, std::vector<lit> &out_learnt, size_t &out_btlevel) noexcept
    {
        std::set<var> seen;
        int counter = 0; // this is the number of variables of the current decision level that have already been seen..
        lit p;
        std::vector<lit> p_reason;
        cnfl.get_reason(p, p_reason);
        out_learnt.push_back(p);
        out_btlevel = 0;
        do
        {
            // trace reason for 'p'..
            for (const auto &q : p_reason) // the order in which these literals are visited is not relevant..
                if (seen.insert(variable(q)).second)
                {
                    assert(value(q) == True); // this literal should have propagated the clause..
                    if (level[variable(q)] == decision_level())
                        counter++;
                    else if (level[variable(q)] > 0) // exclude variables from decision level 0..
                    {
                        out_learnt.push_back(!q); // this literal has been assigned in a previous decision level..
                        out_btlevel = std::max(out_btlevel, level[variable(q)]);
                    }
                }
            // select next literal to look at..
            do
            {
                p = trail.back();
                assert(level[variable(p)] == decision_level()); // this variable must have been assigned at the current decision level..
                if (reason[variable(p)])                        // 'p' can be the asserting literal..
                {
                    p_reason.clear();
                    reason[variable(p)]->get_reason(p, p_reason);
                }
                pop_one();
            } while (!seen.count(variable(p)));
            counter--;
        } while (counter > 0);
        // 'p' is now the first Unique Implication Point (UIP), possibly the asserting literal, that led to the conflict..
        assert(value(p) == Undefined);
        assert(std::all_of(out_learnt.begin() + 1, out_learnt.end(), [this](const lit &lt) { return value(lt) == False; })); // all these literals must have been assigned as false for propagating 'p'..
        out_learnt[0] = !p;
    }

    void sat_core::record(const std::vector<lit> &lits) noexcept
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
            std::sort(c_lits.begin() + 1, c_lits.end(), [this](const lit &a, const lit &b) { return level[variable(a)] > level[variable(b)]; });
            clause *c = new clause(*this, c_lits);
            bool e = enqueue(c_lits[0], c);
            assert(e);
            constrs.push_back(c);
        }
    }

    bool sat_core::enqueue(const lit &p, constr *const c) noexcept
    {
        if (lbool val = value(p); val != Undefined)
            return val;
        else
        {
            assigns[variable(p)] = sign(p);
            level[variable(p)] = decision_level();
            reason[variable(p)] = c;
            trail.push_back(p);
            prop_q.push(p);
            // we notify the listeners that a listening variable has been assigned..
            if (const auto at_p = listening.find(variable(p)); at_p != listening.end())
            {
                for (const auto &l : at_p->second)
                    l->sat_value_change(variable(p));
                if (root_level()) // since this variable will no more be assigned, we can perform some cleanings..
                    listening.erase(at_p);
            }
            return true;
        }
    }

    void sat_core::pop_one() noexcept
    {
        const var v = variable(trail.back());
        assigns[v] = Undefined;
        level[v] = 0;
        reason[v] = nullptr;
        trail.pop_back();
        if (const auto at_v = listening.find(v); at_v != listening.end())
            for (const auto &l : at_v->second)
                l->sat_value_change(v);
    }
} // namespace smt