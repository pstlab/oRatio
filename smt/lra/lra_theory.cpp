#include "lra_theory.h"
#include "row.h"
#include "assertion.h"
#include "lra_value_listener.h"
#include <algorithm>
#include <cassert>

namespace smt
{

lra_theory::lra_theory(sat_core &sat) : theory(sat) {}

lra_theory::~lra_theory() {}

var lra_theory::new_var()
{
    // we create a new arithmetic variable..
    const var id = vals.size();
    bounds.push_back({rational::NEGATIVE_INFINITY, nullptr}); // we set the lower bound at -inf..
    bounds.push_back({rational::POSITIVE_INFINITY, nullptr}); // we set the upper bound at +inf..
    vals.push_back(rational::ZERO);                           // we set the current value at 0..
    exprs.emplace("x" + std::to_string(id), id);
    a_watches.resize(vals.size());
    t_watches.resize(vals.size());
#ifdef OPTIMIZE_FOR_NATIVE_ARCH
    t_mutexes.resize(vals.size());
#endif
    return id;
}

var lra_theory::new_var(const lin &l)
{
    // we create, if needed, a new arithmetic variable which is equal to the given linear expression..
    assert(sat.root_level());
    const std::string s_expr = l.to_string();
    const auto at_expr = exprs.find(s_expr);
    if (at_expr != exprs.end()) // the expression already exists..
        return at_expr->second;
    else // we need to create a new slack variable..
    {
        const var slack = new_var();
        exprs.emplace(s_expr, slack);
        vals[slack] = value(l); // we set the initial value of the new slack variable..
        new_row(slack, l);      // we add a new row into the tableau..
        return slack;
    }
}

var lra_theory::new_lt(const lin &left, const lin &right)
{
    lin expr = left - right;
    std::vector<var> vars;
    vars.reserve(expr.vars.size());
    for (const auto &term : expr.vars)
        vars.push_back(term.first);
    for (const auto &v : vars)
    {
        const auto at_v = tableau.find(v);
        if (at_v != tableau.end())
        {
            rational c = expr.vars.at(v);
            expr.vars.erase(v);
            expr += at_v->second->l * c;
        }
    }

    const inf_rational c_right = inf_rational(-expr.known_term, -1);
    expr.known_term = 0;

    if (ub(expr) <= c_right) // the constraint is already satisfied..
        return TRUE_var;
    else if (lb(expr) > c_right) // the constraint is unsatisfable..
        return FALSE_var;

    const var slack = new_var(expr);
    const std::string s_assertion = "x" + std::to_string(slack) + " <= " + c_right.to_string();
    const auto at_asrt = s_asrts.find(s_assertion);
    if (at_asrt != s_asrts.end()) // this assertion already exists..
        return at_asrt->second;
    else
    {
        const var ctr = sat.new_var();
        bind(ctr);
        s_asrts.emplace(s_assertion, ctr);
        v_asrts[ctr].push_back(new assertion(*this, op::leq, ctr, slack, c_right));
        return ctr;
    }
}

var lra_theory::new_leq(const lin &left, const lin &right)
{
    lin expr = left - right;
    std::vector<var> vars;
    vars.reserve(expr.vars.size());
    for (const auto &term : expr.vars)
        vars.push_back(term.first);
    for (const auto &v : vars)
    {
        const auto at_v = tableau.find(v);
        if (at_v != tableau.end())
        {
            rational c = expr.vars.at(v);
            expr.vars.erase(v);
            expr += at_v->second->l * c;
        }
    }

    const inf_rational c_right = -expr.known_term;
    expr.known_term = 0;

    if (ub(expr) <= c_right) // the constraint is already satisfied..
        return TRUE_var;
    else if (lb(expr) > c_right) // the constraint is unsatisfable..
        return FALSE_var;

    const var slack = new_var(expr);
    const std::string s_assertion = "x" + std::to_string(slack) + " <= " + c_right.to_string();
    const auto at_asrt = s_asrts.find(s_assertion);
    if (at_asrt != s_asrts.end()) // this assertion already exists..
        return at_asrt->second;
    else
    {
        const var ctr = sat.new_var();
        bind(ctr);
        s_asrts.emplace(s_assertion, ctr);
        v_asrts[ctr].push_back(new assertion(*this, op::leq, ctr, slack, c_right));
        return ctr;
    }
}

var lra_theory::new_geq(const lin &left, const lin &right)
{
    lin expr = left - right;
    std::vector<var> vars;
    vars.reserve(expr.vars.size());
    for (const auto &term : expr.vars)
        vars.push_back(term.first);
    for (const auto &v : vars)
    {
        const auto at_v = tableau.find(v);
        if (at_v != tableau.end())
        {
            rational c = expr.vars.at(v);
            expr.vars.erase(v);
            expr += at_v->second->l * c;
        }
    }

    const inf_rational c_right = -expr.known_term;
    expr.known_term = 0;

    if (lb(expr) >= c_right) // the constraint is already satisfied..
        return TRUE_var;
    else if (ub(expr) < c_right) // the constraint is unsatisfable..
        return FALSE_var;

    const var slack = new_var(expr);
    const std::string s_assertion = "x" + std::to_string(slack) + " >= " + c_right.to_string();
    const auto at_asrt = s_asrts.find(s_assertion);
    if (at_asrt != s_asrts.end()) // this assertion already exists..
        return at_asrt->second;
    else
    {
        const var ctr = sat.new_var();
        bind(ctr);
        s_asrts.emplace(s_assertion, ctr);
        v_asrts[ctr].push_back(new assertion(*this, op::geq, ctr, slack, c_right));
        return ctr;
    }
}

var lra_theory::new_gt(const lin &left, const lin &right)
{
    lin expr = left - right;
    std::vector<var> vars;
    vars.reserve(expr.vars.size());
    for (const auto &term : expr.vars)
        vars.push_back(term.first);
    for (const auto &v : vars)
    {
        const auto at_v = tableau.find(v);
        if (at_v != tableau.end())
        {
            rational c = expr.vars.at(v);
            expr.vars.erase(v);
            expr += at_v->second->l * c;
        }
    }

    const inf_rational c_right = inf_rational(-expr.known_term, 1);
    expr.known_term = 0;

    if (lb(expr) >= c_right) // the constraint is already satisfied..
        return TRUE_var;
    else if (ub(expr) < c_right) // the constraint is unsatisfable..
        return FALSE_var;

    const var slack = new_var(expr);
    const std::string s_assertion = "x" + std::to_string(slack) + " >= " + c_right.to_string();
    const auto at_asrt = s_asrts.find(s_assertion);
    if (at_asrt != s_asrts.end()) // this assertion already exists..
        return at_asrt->second;
    else
    {
        const var ctr = sat.new_var();
        bind(ctr);
        s_asrts.emplace(s_assertion, ctr);
        v_asrts[ctr].push_back(new assertion(*this, op::geq, ctr, slack, c_right));
        return ctr;
    }
}

bool lra_theory::lt(const lin &left, const lin &right, const var &p)
{
    lin expr = left - right;
    std::vector<var> vars;
    vars.reserve(expr.vars.size());
    for (const auto &term : expr.vars)
        vars.push_back(term.first);
    for (const auto &v : vars)
    {
        const auto at_v = tableau.find(v);
        if (at_v != tableau.end())
        {
            rational c = expr.vars.at(v);
            expr.vars.erase(v);
            expr += at_v->second->l * c;
        }
    }

    const inf_rational c_right = inf_rational(-expr.known_term, -1);
    expr.known_term = 0;

    if (ub(expr) <= c_right) // the constraint is already satisfied..
        return sat.eq(p, TRUE_var);
    else if (lb(expr) > c_right) // the constraint is unsatisfable..
        return sat.eq(p, FALSE_var);

    const var slack = new_var(expr);
    const std::string s_assertion = "x" + std::to_string(slack) + " <= " + c_right.to_string();
    const auto at_asrt = s_asrts.find(s_assertion);
    if (at_asrt != s_asrts.end()) // this assertion already exists..
        return sat.eq(p, at_asrt->second);
    else
        switch (sat.value(p))
        {
        case True:
        {
            std::vector<lit> cnfl;
            return assert_upper(slack, c_right, p, cnfl);
        }
        case False:
        {
            std::vector<lit> cnfl;
            return assert_lower(slack, c_right, lit(p, false), cnfl);
        }
        default:
            bind(p);
            s_asrts.emplace(s_assertion, p);
            v_asrts[p].push_back(new assertion(*this, op::leq, p, slack, c_right));
            return true;
        }
}

bool lra_theory::leq(const lin &left, const lin &right, const var &p)
{
    lin expr = left - right;
    std::vector<var> vars;
    vars.reserve(expr.vars.size());
    for (const auto &term : expr.vars)
        vars.push_back(term.first);
    for (const auto &v : vars)
    {
        const auto at_v = tableau.find(v);
        if (at_v != tableau.end())
        {
            rational c = expr.vars.at(v);
            expr.vars.erase(v);
            expr += at_v->second->l * c;
        }
    }

    const inf_rational c_right = -expr.known_term;
    expr.known_term = 0;

    if (ub(expr) <= c_right) // the constraint is already satisfied..
        return sat.eq(p, TRUE_var);
    else if (lb(expr) > c_right) // the constraint is unsatisfable..
        return sat.eq(p, FALSE_var);

    const var slack = new_var(expr);
    const std::string s_assertion = "x" + std::to_string(slack) + " <= " + c_right.to_string();
    const auto at_asrt = s_asrts.find(s_assertion);
    if (at_asrt != s_asrts.end()) // this assertion already exists..
        return sat.eq(p, at_asrt->second);
    else
        switch (sat.value(p))
        {
        case True:
        {
            std::vector<lit> cnfl;
            return assert_upper(slack, c_right, p, cnfl);
        }
        case False:
        {
            std::vector<lit> cnfl;
            return assert_lower(slack, c_right, lit(p, false), cnfl);
        }
        default:
            bind(p);
            s_asrts.emplace(s_assertion, p);
            v_asrts[p].push_back(new assertion(*this, op::leq, p, slack, c_right));
            return true;
        }
}

bool lra_theory::geq(const lin &left, const lin &right, const var &p)
{
    lin expr = left - right;
    std::vector<var> vars;
    vars.reserve(expr.vars.size());
    for (const auto &term : expr.vars)
        vars.push_back(term.first);
    for (const auto &v : vars)
    {
        const auto at_v = tableau.find(v);
        if (at_v != tableau.end())
        {
            rational c = expr.vars.at(v);
            expr.vars.erase(v);
            expr += at_v->second->l * c;
        }
    }

    const inf_rational c_right = -expr.known_term;
    expr.known_term = 0;

    if (lb(expr) >= c_right) // the constraint is already satisfied..
        return sat.eq(p, TRUE_var);
    else if (ub(expr) < c_right) // the constraint is unsatisfable..
        return sat.eq(p, FALSE_var);

    const var slack = new_var(expr);
    const std::string s_assertion = "x" + std::to_string(slack) + " >= " + c_right.to_string();
    const auto at_asrt = s_asrts.find(s_assertion);
    if (at_asrt != s_asrts.end()) // this assertion already exists..
        return sat.eq(p, at_asrt->second);
    else
        switch (sat.value(p))
        {
        case True:
        {
            std::vector<lit> cnfl;
            return assert_lower(slack, c_right, p, cnfl);
        }
        case False:
        {
            std::vector<lit> cnfl;
            return assert_upper(slack, c_right, lit(p, false), cnfl);
        }
        default:
            bind(p);
            s_asrts.emplace(s_assertion, p);
            v_asrts[p].push_back(new assertion(*this, op::geq, p, slack, c_right));
            return true;
        }
}

bool lra_theory::gt(const lin &left, const lin &right, const var &p)
{
    lin expr = left - right;
    std::vector<var> vars;
    vars.reserve(expr.vars.size());
    for (const auto &term : expr.vars)
        vars.push_back(term.first);
    for (const auto &v : vars)
        if (const auto at_v = tableau.find(v); at_v != tableau.end())
        {
            rational c = expr.vars.at(v);
            expr.vars.erase(v);
            expr += at_v->second->l * c;
        }

    const inf_rational c_right = inf_rational(-expr.known_term, 1);
    expr.known_term = 0;

    if (lb(expr) >= c_right) // the constraint is already satisfied..
        return sat.eq(p, TRUE_var);
    else if (ub(expr) < c_right) // the constraint is unsatisfable..
        return sat.eq(p, FALSE_var);

    const var slack = new_var(expr);
    const std::string s_assertion = "x" + std::to_string(slack) + " >= " + c_right.to_string();
    const auto at_asrt = s_asrts.find(s_assertion);
    if (at_asrt != s_asrts.end()) // this assertion already exists..
        return sat.eq(p, at_asrt->second);
    else
        switch (sat.value(p))
        {
        case True:
        {
            std::vector<lit> cnfl;
            return assert_lower(slack, c_right, p, cnfl);
        }
        case False:
        {
            std::vector<lit> cnfl;
            return assert_upper(slack, c_right, lit(p, false), cnfl);
        }
        default:
            bind(p);
            s_asrts.emplace(s_assertion, p);
            v_asrts[p].push_back(new assertion(*this, op::geq, p, slack, c_right));
            return true;
        }
}

bool lra_theory::propagate(const lit &p, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    for (const auto &a : v_asrts.at(p.get_var()))
        if (p.get_sign()) // the assertion is direct..
        {
            if (!((a->o == op::leq) ? assert_upper(a->x, a->v, p, cnfl) : assert_lower(a->x, a->v, p, cnfl)))
                return false;
        }
        else // the assertion is negated..
        {
            if (!((a->o == op::leq) ? assert_lower(a->x, a->v + inf_rational(rational::ZERO, rational::ONE), p, cnfl) : assert_upper(a->x, a->v - inf_rational(rational::ZERO, rational::ONE), p, cnfl)))
                return false;
        }

    return true;
}

bool lra_theory::check(std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    while (true)
    {
        const auto &x_i_it = std::find_if(tableau.begin(), tableau.end(), [this](const std::pair<var, row *> &v) { return value(v.first) < lb(v.first) || value(v.first) > ub(v.first); });
        if (x_i_it == tableau.end())
            return true;
        // the current value of the x_i variable is out of its bounds..
        const var x_i = (*x_i_it).first;
        // the flawed row..
        const row *f_row = (*x_i_it).second;
        if (value(x_i) < lb(x_i))
        {
            const auto &x_j_it = std::find_if(f_row->l.vars.begin(), f_row->l.vars.end(), [f_row, this](const std::pair<var, inf_rational> &v) { return (f_row->l.vars.at(v.first).is_positive() && value(v.first) < ub(v.first)) || (f_row->l.vars.at(v.first).is_negative() && value(v.first) > lb(v.first)); });
            if (x_j_it != f_row->l.vars.end()) // var x_j can be used to increase the value of x_i..
                pivot_and_update(x_i, (*x_j_it).first, lb(x_i));
            else // we generate an explanation for the conflict..
            {
                for (const auto &term : f_row->l.vars)
                    if (term.second.is_positive())
                        cnfl.push_back(!*bounds[lra_theory::ub_index(term.first)].reason);
                    else if (term.second.is_negative())
                        cnfl.push_back(!*bounds[lra_theory::lb_index(term.first)].reason);
                cnfl.push_back(!*bounds[lra_theory::lb_index(x_i)].reason);
                return false;
            }
        }
        else if (value(x_i) > ub(x_i))
        {
            const auto &x_j_it = std::find_if(f_row->l.vars.begin(), f_row->l.vars.end(), [f_row, this](const std::pair<var, inf_rational> &v) { return (f_row->l.vars.at(v.first).is_negative() && value(v.first) < ub(v.first)) || (f_row->l.vars.at(v.first).is_positive() && value(v.first) > lb(v.first)); });
            if (x_j_it != f_row->l.vars.end()) // var x_j can be used to decrease the value of x_i..
                pivot_and_update(x_i, (*x_j_it).first, ub(x_i));
            else // we generate an explanation for the conflict..
            {
                for (const auto &term : f_row->l.vars)
                    if (term.second.is_positive())
                        cnfl.push_back(!*bounds[lra_theory::lb_index(term.first)].reason);
                    else if (term.second.is_negative())
                        cnfl.push_back(!*bounds[lra_theory::ub_index(term.first)].reason);
                cnfl.push_back(!*bounds[lra_theory::ub_index(x_i)].reason);
                return false;
            }
        }
    }
}

void lra_theory::push() { layers.push_back(std::unordered_map<size_t, bound>()); }

void lra_theory::pop()
{
    // we restore the variables' bounds and their reason..
    for (const auto &b : layers.back())
    {
        delete bounds[b.first].reason;
        bounds[b.first] = b.second;
    }
    layers.pop_back();
}

bool lra_theory::assert_lower(const var &x_i, const inf_rational &val, const lit &p, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    if (val <= lb(x_i))
        return true;
    else if (val > ub(x_i))
    {
        cnfl.push_back(!p);                             // either the literal 'p' is false ..
        cnfl.push_back(!*bounds[ub_index(x_i)].reason); // or what asserted the upper bound is false..
        return false;
    }
    else
    {
        if (!layers.empty() && !layers.back().count(lb_index(x_i)))
            layers.back().insert({lb_index(x_i), {lb(x_i), bounds[lb_index(x_i)].reason}});
        bounds[lb_index(x_i)] = {val, new lit(p.get_var(), p.get_sign())};

        if (vals[x_i] < val && !is_basic(x_i))
            update(x_i, val);

        // unate propagation..
        for (const auto &c : a_watches[x_i])
            if (!c->propagate_lb(x_i, cnfl))
                return false;
        // bound propagation..
        for (const auto &c : t_watches[x_i])
            if (!c->propagate_lb(x_i, cnfl))
                return false;

        return true;
    }
}

bool lra_theory::assert_upper(const var &x_i, const inf_rational &val, const lit &p, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    if (val >= ub(x_i))
        return true;
    else if (val < lb(x_i))
    {
        cnfl.push_back(!p);                             // either the literal 'p' is false ..
        cnfl.push_back(!*bounds[lb_index(x_i)].reason); // or what asserted the lower bound is false..
        return false;
    }
    else
    {
        if (!layers.empty() && !layers.back().count(ub_index(x_i)))
            layers.back().insert({ub_index(x_i), {ub(x_i), bounds[ub_index(x_i)].reason}});
        bounds[ub_index(x_i)] = {val, new lit(p.get_var(), p.get_sign())};

        if (vals[x_i] > val && !is_basic(x_i))
            update(x_i, val);

        // unate propagation..
        for (const auto &c : a_watches[x_i])
            if (!c->propagate_ub(x_i, cnfl))
                return false;
        // bound propagation..
        for (const auto &c : t_watches[x_i])
            if (!c->propagate_ub(x_i, cnfl))
                return false;

        return true;
    }
}

void lra_theory::update(const var &x_i, const inf_rational &v)
{
    assert(!is_basic(x_i) && "x_i should be a non-basic variable..");
    for (const auto &c : t_watches[x_i])
    {
        // x_j = x_j + a_ji(v - x_i)..
        vals[c->x] += c->l.vars.at(x_i) * (v - vals[x_i]);
        const auto at_c_x = listening.find(c->x);
        if (at_c_x != listening.end())
            for (const auto &l : at_c_x->second)
                l->lra_value_change(c->x);
    }
    // x_i = v..
    vals[x_i] = v;
    const auto at_x_i = listening.find(x_i);
    if (at_x_i != listening.end())
        for (const auto &l : at_x_i->second)
            l->lra_value_change(x_i);
}

void lra_theory::pivot_and_update(const var &x_i, const var &x_j, const inf_rational &v)
{
    assert(is_basic(x_i) && "x_i should be a basic variable..");
    assert(!is_basic(x_j) && "x_j should be a non-basic variable..");
    assert(tableau.at(x_i)->l.vars.count(x_j));

    const inf_rational theta = (v - vals[x_i]) / tableau.at(x_i)->l.vars.at(x_j);
    assert(!theta.is_infinite());

    // x_i = v
    vals[x_i] = v;
    if (const auto at_x_i = listening.find(x_i); at_x_i != listening.end())
        for (const auto &l : at_x_i->second)
            l->lra_value_change(x_i);

    // x_j += theta
    vals[x_j] += theta;
    if (const auto at_x_j = listening.find(x_j); at_x_j != listening.end())
        for (const auto &l : at_x_j->second)
            l->lra_value_change(x_j);

    for (const auto &c : t_watches[x_j])
        if (c->x != x_i)
        { // x_k += a_kj * theta..
            vals[c->x] += c->l.vars.at(x_j) * theta;
            if (const auto at_x_c = listening.find(c->x); at_x_c != listening.end())
                for (const auto &l : at_x_c->second)
                    l->lra_value_change(c->x);
        }

    pivot(x_i, x_j);
}

void lra_theory::pivot(const var x_i, const var x_j)
{
    // the exiting row..
    row *ex_row = tableau.at(x_i);
    lin expr = std::move(ex_row->l);
    tableau.erase(x_i);
    // we remove the row from the watches..
    for (const auto &c : expr.vars)
        t_watches[c.first].erase(ex_row);
    delete ex_row;

    const rational c = expr.vars.at(x_j);
    expr.vars.erase(x_j);
    expr /= -c;
    expr.vars.emplace(x_i, rational::ONE / c);

    // these are the rows in which x_j appears..
    std::unordered_set<row *> x_j_watches;
    std::swap(x_j_watches, t_watches[x_j]);
    for (const auto &r : x_j_watches)
    { // 'r' is a row in which 'x_j' appears..
#ifdef OPTIMIZE_FOR_NATIVE_ARCH
        sat.get_thread_pool().enqueue([this, x_j, expr, r] {
#endif
            rational cc = r->l.vars.at(x_j);
            r->l.vars.erase(x_j);
            for (const auto &term : expr.vars)
                if (const auto trm_it = r->l.vars.find(term.first); trm_it == r->l.vars.end())
                { // we are adding a new term to 'r'..
                    r->l.vars.emplace(term.first, term.second * cc);
#ifdef OPTIMIZE_FOR_NATIVE_ARCH
                    {
                        std::lock_guard<std::mutex> lock(t_mutexes[term.first]);
#endif
                        t_watches[term.first].emplace(r);
#ifdef OPTIMIZE_FOR_NATIVE_ARCH
                    }
#endif
                }
                else
                { // we are updating an existing term of 'r'..
                    assert(trm_it->first == term.first);
                    trm_it->second += term.second * cc;
                    if (trm_it->second == rational::ZERO)
                    { // the updated term's coefficient has become equal to zero, hence we can remove the term..
                        r->l.vars.erase(trm_it);
                        t_watches[term.first].erase(r);
#ifdef OPTIMIZE_FOR_NATIVE_ARCH
                        {
                            std::lock_guard<std::mutex> lock(t_mutexes[term.first]);
#endif
                            t_watches[term.first].erase(r);
#ifdef OPTIMIZE_FOR_NATIVE_ARCH
                        }
#endif
                    }
                }
            r->l.known_term += expr.known_term * cc;
#ifdef OPTIMIZE_FOR_NATIVE_ARCH
        });
#endif
    }
#ifdef OPTIMIZE_FOR_NATIVE_ARCH
    sat.get_thread_pool().join();
#endif

    // we add a new row into the tableau..
    new_row(x_j, expr);
}

void lra_theory::new_row(const var &x, const lin &l)
{
    row *r = new row(*this, x, l);
    tableau.emplace(x, r);
    for (const auto &term : l.vars)
        t_watches[term.first].emplace(r);
}

std::string lra_theory::to_string()
{
    std::string la;
    la += "{ \"vars\" : [";
    for (size_t i = 0; i < vals.size(); ++i)
    {
        if (i)
            la += ", ";
        la += "{ \"name\" : \"x" + std::to_string(i) + "\", \"value\" : " + value(i).to_string();
        if (!lb(i).is_negative_infinite())
            la += ", \"lb\" : " + lb(i).to_string();
        if (!ub(i).is_positive_infinite())
            la += ", \"ub\" : " + ub(i).to_string();
        la += "}";
    }
    la += "], \"asrts\" : [";
    for (auto it = v_asrts.cbegin(); it != v_asrts.cend(); ++it)
    {
        if (it != v_asrts.cbegin())
            la += ", ";
        la += "[";
        for (auto a_it = it->second.cbegin(); a_it != it->second.cend(); ++a_it)
        {
            if (a_it != it->second.cbegin())
                la += ", ";
            la += (*a_it)->to_string();
        }
        la += "]";
    }
    la += "], \"tableau\" : [";
    for (auto it = tableau.cbegin(); it != tableau.cend(); ++it)
    {
        if (it != tableau.cbegin())
            la += ", ";
        la += it->second->to_string();
    }
    la += "]";
    la += "}";
    return la;
}
} // namespace smt