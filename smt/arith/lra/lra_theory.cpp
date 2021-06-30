#include "lra_theory.h"
#include "lra_constraint.h"
#include "lra_value_listener.h"
#include <algorithm>
#include <cassert>

namespace smt
{
    SMT_EXPORT lra_theory::lra_theory(sat_core &sat) : theory(sat) {}
    SMT_EXPORT lra_theory::~lra_theory() {}

    SMT_EXPORT var lra_theory::new_var() noexcept
    {
        // we create a new arithmetic variable..
        const var id = vals.size();
        c_bounds.push_back({inf_rational(rational::NEGATIVE_INFINITY), TRUE_lit}); // we set the lower bound at -inf..
        c_bounds.push_back({inf_rational(rational::POSITIVE_INFINITY), TRUE_lit}); // we set the upper bound at +inf..
        vals.emplace_back(rational::ZERO);                                         // we set the current value at 0..
        exprs.emplace("x" + std::to_string(id), id);
        a_watches.resize(vals.size());
        t_watches.resize(vals.size());
#ifdef PARALLELIZE
        t_mtxs.resize(vals.size());
#endif
        return id;
    }

    SMT_EXPORT var lra_theory::new_var(const lin &l) noexcept
    { // we create, if needed, a new arithmetic variable which is equal to the given linear expression..
        const std::string s_expr = to_string(l);
        if (const auto at_expr = exprs.find(s_expr); at_expr != exprs.cend()) // the expression already exists..
            return at_expr->second;
        else
        { // we need to create a new slack variable..
            assert(sat.root_level());
            const var slack = new_var();
            exprs.emplace(s_expr, slack);
            c_bounds[lb_index(slack)] = {lb(l), TRUE_lit}; // we set the lower bound at the lower bound of the given linear expression..
            c_bounds[ub_index(slack)] = {ub(l), TRUE_lit}; // we set the upper bound at the upper bound of the given linear expression..
            vals[slack] = value(l);                        // we set the initial value of the new slack variable at the value of the given linear expression..
            new_row(slack, l);                             // we add a new row into the tableau..
            return slack;
        }
    }

    SMT_EXPORT lit lra_theory::new_lt(const lin &left, const lin &right) noexcept
    {
        lin expr = left - right;
        std::vector<var> vars;
        vars.reserve(expr.vars.size());
        for (const auto &[v, c] : expr.vars)
            vars.push_back(v);
        for (const auto &v : vars)
            if (const auto at_v = tableau.find(v); at_v != tableau.cend())
            {
                rational c = expr.vars[v];
                expr.vars.erase(v);
                expr += at_v->second->l * c;
            }

        const inf_rational c_right = inf_rational(-expr.known_term, -1);
        expr.known_term = rational::ZERO;

        if (ub(expr) <= c_right)
            return TRUE_lit; // the constraint is already satisfied..
        else if (lb(expr) > c_right)
            return FALSE_lit; // the constraint is unsatisfable..

        // we create a slack variable from the current expression (notice that the variable can be reused)..
        const var slack = new_var(expr);
        if (ub(slack) <= c_right)
            return TRUE_lit; // the constraint is already satisfied..
        else if (lb(slack) > c_right)
            return FALSE_lit; // the constraint is unsatisfable..
        const std::string s_assertion = "x" + std::to_string(slack) + " <= " + to_string(c_right);
        if (const auto at_asrt = s_asrts.find(s_assertion); at_asrt != s_asrts.cend()) // this assertion already exists..
            return at_asrt->second;
        else
        { // we need to create a new control variable..
            const auto ctr = sat.new_var();
            const lit ctr_lit(ctr);
            bind(ctr);
            s_asrts.emplace(s_assertion, ctr_lit);
            v_asrts[ctr].push_back(new assertion(*this, op::leq, ctr_lit, slack, c_right));
            return ctr_lit;
        }
    }

    SMT_EXPORT lit lra_theory::new_leq(const lin &left, const lin &right) noexcept
    {
        lin expr = left - right;
        std::vector<var> vars;
        vars.reserve(expr.vars.size());
        for (const auto &[v, c] : expr.vars)
            vars.push_back(v);
        for (const auto &v : vars)
            if (const auto at_v = tableau.find(v); at_v != tableau.cend())
            {
                rational c = expr.vars[v];
                expr.vars.erase(v);
                expr += at_v->second->l * c;
            }

        const inf_rational c_right = -inf_rational(expr.known_term);
        expr.known_term = rational::ZERO;

        if (ub(expr) <= c_right)
            return TRUE_lit; // the constraint is already satisfied..
        else if (lb(expr) > c_right)
            return FALSE_lit; // the constraint is unsatisfable..

        // we create a slack variable from the current expression (notice that the variable can be reused)..
        const var slack = new_var(expr);
        if (ub(slack) <= c_right)
            return TRUE_lit; // the constraint is already satisfied..
        else if (lb(slack) > c_right)
            return FALSE_lit; // the constraint is unsatisfable..
        const std::string s_assertion = "x" + std::to_string(slack) + " <= " + to_string(c_right);
        if (const auto at_asrt = s_asrts.find(s_assertion); at_asrt != s_asrts.cend()) // this assertion already exists..
            return at_asrt->second;
        else
        { // we need to create a new control variable..
            const auto ctr = sat.new_var();
            const lit ctr_lit(ctr);
            bind(ctr);
            s_asrts.emplace(s_assertion, ctr_lit);
            v_asrts[ctr].push_back(new assertion(*this, op::leq, ctr_lit, slack, c_right));
            return ctr_lit;
        }
    }

    SMT_EXPORT lit lra_theory::new_geq(const lin &left, const lin &right) noexcept
    {
        lin expr = left - right;
        std::vector<var> vars;
        vars.reserve(expr.vars.size());
        for (const auto &[v, c] : expr.vars)
            vars.push_back(v);
        for (const auto &v : vars)
            if (const auto at_v = tableau.find(v); at_v != tableau.cend())
            {
                rational c = expr.vars[v];
                expr.vars.erase(v);
                expr += at_v->second->l * c;
            }

        const inf_rational c_right = -inf_rational(expr.known_term);
        expr.known_term = rational::ZERO;

        if (lb(expr) >= c_right)
            return TRUE_lit; // the constraint is already satisfied..
        else if (ub(expr) < c_right)
            return FALSE_lit; // the constraint is unsatisfable..

        // we create a slack variable from the current expression (notice that the variable can be reused)..
        const var slack = new_var(expr);
        if (lb(slack) >= c_right)
            return TRUE_lit; // the constraint is already satisfied..
        else if (ub(slack) < c_right)
            return FALSE_lit; // the constraint is unsatisfable..
        const std::string s_assertion = "x" + std::to_string(slack) + " >= " + to_string(c_right);
        if (const auto at_asrt = s_asrts.find(s_assertion); at_asrt != s_asrts.cend()) // this assertion already exists..
            return at_asrt->second;
        else
        { // we need to create a new control variable..
            const auto ctr = sat.new_var();
            const lit ctr_lit(ctr);
            bind(ctr);
            s_asrts.emplace(s_assertion, ctr_lit);
            v_asrts[ctr].push_back(new assertion(*this, op::geq, ctr_lit, slack, c_right));
            return ctr_lit;
        }
    }

    SMT_EXPORT lit lra_theory::new_gt(const lin &left, const lin &right) noexcept
    {
        lin expr = left - right;
        std::vector<var> vars;
        vars.reserve(expr.vars.size());
        for (const auto &[v, c] : expr.vars)
            vars.push_back(v);
        for (const auto &v : vars)
            if (const auto at_v = tableau.find(v); at_v != tableau.cend())
            {
                rational c = expr.vars[v];
                expr.vars.erase(v);
                expr += at_v->second->l * c;
            }

        const inf_rational c_right = inf_rational(-expr.known_term, 1);
        expr.known_term = rational::ZERO;

        if (lb(expr) >= c_right)
            return TRUE_lit; // the constraint is already satisfied..
        else if (ub(expr) < c_right)
            return FALSE_lit; // the constraint is unsatisfable..

        // we create a slack variable from the current expression (notice that the variable can be reused)..
        const var slack = new_var(expr);
        if (lb(slack) >= c_right)
            return TRUE_lit; // the constraint is already satisfied..
        else if (ub(slack) < c_right)
            return FALSE_lit; // the constraint is unsatisfable..
        const std::string s_assertion = "x" + std::to_string(slack) + " >= " + to_string(c_right);
        if (const auto at_asrt = s_asrts.find(s_assertion); at_asrt != s_asrts.cend()) // this assertion already exists..
            return at_asrt->second;
        else
        { // we need to create a new control variable..
            const auto ctr = sat.new_var();
            const lit ctr_lit(ctr);
            bind(ctr);
            s_asrts.emplace(s_assertion, ctr_lit);
            v_asrts[ctr].push_back(new assertion(*this, op::geq, ctr_lit, slack, c_right));
            return ctr_lit;
        }
    }

    SMT_EXPORT bool lra_theory::equates(const lin &l0, const lin &l1) const noexcept
    {
        const auto [l0_lb, l0_ub] = bounds(l0);
        const auto [l1_lb, l1_ub] = bounds(l1);
        return l0_ub >= l1_lb && l0_lb <= l1_ub; // the two intervals intersect..
    }

    bool lra_theory::propagate(const lit &p) noexcept
    {
        assert(cnfl.empty());
        for (const auto &a : v_asrts[variable(p)])
            switch (sat.value(a->b))
            {
            case True: // the assertion is direct..
                if (!((a->o == op::leq) ? assert_upper(a->x, a->v, p) : assert_lower(a->x, a->v, p)))
                    return false;
                break;
            case False: // the assertion is negated..
                if (!((a->o == op::leq) ? assert_lower(a->x, a->v + inf_rational(rational::ZERO, rational::ONE), p) : assert_upper(a->x, a->v - inf_rational(rational::ZERO, rational::ONE), p)))
                    return false;
                break;
            }

        return true;
    }

    bool lra_theory::check() noexcept
    {
        assert(cnfl.empty());
        while (true)
        {
            const auto &x_i_it = std::find_if(tableau.cbegin(), tableau.cend(), [this](const auto &v)
                                              { return value(v.first) < lb(v.first) || value(v.first) > ub(v.first); });
            if (x_i_it == tableau.cend())
                return true;
            // the current value of the x_i variable is out of its c_bounds..
            const var x_i = (*x_i_it).first;
            // the flawed row..
            const row *f_row = (*x_i_it).second;
            if (value(x_i) < lb(x_i))
            {
                const auto &x_j_it = std::find_if(f_row->l.vars.cbegin(), f_row->l.vars.cend(), [f_row, this](const std::pair<var, rational> &v)
                                                  { return (is_positive(f_row->l.vars.at(v.first)) && value(v.first) < ub(v.first)) || (is_negative(f_row->l.vars.at(v.first)) && value(v.first) > lb(v.first)); });
                if (x_j_it != f_row->l.vars.cend()) // var x_j can be used to increase the value of x_i..
                    pivot_and_update(x_i, (*x_j_it).first, lb(x_i));
                else
                { // we generate an explanation for the conflict..
                    for (const auto &[v, c] : f_row->l.vars)
                        if (is_positive(c))
                            cnfl.push_back(!c_bounds[lra_theory::ub_index(v)].reason);
                        else if (is_negative(c))
                            cnfl.push_back(!c_bounds[lra_theory::lb_index(v)].reason);
                    cnfl.push_back(!c_bounds[lra_theory::lb_index(x_i)].reason);
                    return false;
                }
            }
            else if (value(x_i) > ub(x_i))
            {
                const auto &x_j_it = std::find_if(f_row->l.vars.cbegin(), f_row->l.vars.cend(), [f_row, this](const std::pair<var, rational> &v)
                                                  { return (is_negative(f_row->l.vars.at(v.first)) && value(v.first) < ub(v.first)) || (is_positive(f_row->l.vars.at(v.first)) && value(v.first) > lb(v.first)); });
                if (x_j_it != f_row->l.vars.cend()) // var x_j can be used to decrease the value of x_i..
                    pivot_and_update(x_i, (*x_j_it).first, ub(x_i));
                else
                { // we generate an explanation for the conflict..
                    for (const auto &[v, c] : f_row->l.vars)
                        if (is_positive(c))
                            cnfl.push_back(!c_bounds[lra_theory::lb_index(v)].reason);
                        else if (is_negative(c))
                            cnfl.push_back(!c_bounds[lra_theory::ub_index(v)].reason);
                    cnfl.push_back(!c_bounds[lra_theory::ub_index(x_i)].reason);
                    return false;
                }
            }
        }
    }

    void lra_theory::push() noexcept { layers.push_back(std::unordered_map<size_t, bound>()); }

    void lra_theory::pop() noexcept
    {
        // we restore the variables' c_bounds and their reason..
        for (const auto &[v, bnds] : layers.back())
            c_bounds[v] = bnds;
        layers.pop_back();
    }

    bool lra_theory::assert_lower(const var &x_i, const inf_rational &val, const lit &p) noexcept
    {
        assert(cnfl.empty());
        if (val <= lb(x_i))
            return true;
        else if (val > ub(x_i))
        {
            cnfl.push_back(!p);                              // either the literal 'p' is false ..
            cnfl.push_back(!c_bounds[ub_index(x_i)].reason); // or what asserted the upper bound is false..
            return false;
        }
        else
        {
            if (!layers.empty() && !layers.back().count(lb_index(x_i)))
                layers.back().insert({lb_index(x_i), {lb(x_i), c_bounds[lb_index(x_i)].reason}});
            c_bounds[lb_index(x_i)] = {val, p};

            if (vals[x_i] < val && !is_basic(x_i))
                update(x_i, val); // we set the value of 'x_i' to 'val' and update all the basic variables which are related to 'x_i' by the tableau..

            // unate propagation..
            for (const auto &c : a_watches[x_i])
                if (!c->propagate_lb(x_i))
                    return false;
            // bound propagation..
            for (const auto &c : t_watches[x_i])
                if (!c->propagate_lb(x_i))
                    return false;

            return true;
        }
    }

    bool lra_theory::assert_upper(const var &x_i, const inf_rational &val, const lit &p) noexcept
    {
        assert(cnfl.empty());
        if (val >= ub(x_i))
            return true;
        else if (val < lb(x_i))
        {
            cnfl.push_back(!p);                              // either the literal 'p' is false ..
            cnfl.push_back(!c_bounds[lb_index(x_i)].reason); // or what asserted the lower bound is false..
            return false;
        }
        else
        {
            if (!layers.empty() && !layers.back().count(ub_index(x_i)))
                layers.back().insert({ub_index(x_i), {ub(x_i), c_bounds[ub_index(x_i)].reason}});
            c_bounds[ub_index(x_i)] = {val, p};

            if (vals[x_i] > val && !is_basic(x_i))
                update(x_i, val); // we set the value of 'x_i' to 'val' and update all the basic variables which are related to 'x_i' by the tableau..

            // unate propagation..
            for (const auto &c : a_watches[x_i])
                if (!c->propagate_ub(x_i))
                    return false;
            // bound propagation..
            for (const auto &c : t_watches[x_i])
                if (!c->propagate_ub(x_i))
                    return false;

            return true;
        }
    }

    void lra_theory::update(const var &x_i, const inf_rational &v) noexcept
    {
        assert(!is_basic(x_i) && "x_i should be a non-basic variable..");
        // the tableau rows containing 'x_i' as a non-basic variable..
        for (const auto &c : t_watches[x_i])
        { // x_j = x_j + a_ji(v - x_i)..
            vals[c->x] += c->l.vars.at(x_i) * (v - vals[x_i]);
            if (const auto at_c_x = listening.find(c->x); at_c_x != listening.cend())
                for (const auto &l : at_c_x->second)
                    l->lra_value_change(c->x);
        }
        // x_i = v..
        vals[x_i] = v;
        if (const auto at_x_i = listening.find(x_i); at_x_i != listening.cend())
            for (const auto &l : at_x_i->second)
                l->lra_value_change(x_i);
    }

    void lra_theory::pivot_and_update(const var &x_i, const var &x_j, const inf_rational &v) noexcept
    {
        assert(is_basic(x_i) && "x_i should be a basic variable..");
        assert(!is_basic(x_j) && "x_j should be a non-basic variable..");
        assert(tableau.at(x_i)->l.vars.count(x_j));

        const inf_rational theta = (v - vals[x_i]) / tableau.at(x_i)->l.vars.at(x_j);
        assert(!is_infinite(theta));

        // x_i = v
        vals[x_i] = v;
        if (const auto at_x_i = listening.find(x_i); at_x_i != listening.cend())
            for (const auto &l : at_x_i->second)
                l->lra_value_change(x_i);

        // x_j += theta
        vals[x_j] += theta;
        if (const auto at_x_j = listening.find(x_j); at_x_j != listening.cend())
            for (const auto &l : at_x_j->second)
                l->lra_value_change(x_j);

        // the tableau rows containing 'x_j' as a non-basic variable..
        for (const auto &c : t_watches[x_j])
            if (c->x != x_i)
            { // x_k += a_kj * theta..
                vals[c->x] += c->l.vars.at(x_j) * theta;
                if (const auto at_x_c = listening.find(c->x); at_x_c != listening.cend())
                    for (const auto &l : at_x_c->second)
                        l->lra_value_change(c->x);
            }

        pivot(x_i, x_j);
    }

    void lra_theory::pivot(const var x_i, const var x_j) noexcept
    {
        // the exiting row..
        row *ex_row = tableau[x_i];
        lin expr = std::move(ex_row->l);
        tableau.erase(x_i);
        // we remove the row from the watches..
        for (const auto &[v, c] : expr.vars)
            t_watches[v].erase(ex_row);
        delete ex_row;

        const rational cf = expr.vars[x_j];
        expr.vars.erase(x_j);
        expr /= -cf;
        expr.vars.emplace(x_i, rational::ONE / cf);

        // these are the rows in which x_j appears..
        std::unordered_set<row *> x_j_watches;
        std::swap(x_j_watches, t_watches[x_j]);
        for (const auto &r : x_j_watches)
#ifdef PARALLELIZE
            sat.get_thread_pool().enqueue([this, x_j, expr, r]
                                          {
                                              rational cc = r->l.vars[x_j];
                                              r->l.vars.erase(x_j);
                                              for (const auto &[v, c] : std::map<const var, rational>(expr.vars))
                                                  if (const auto trm_it = r->l.vars.find(v); trm_it == r->l.vars.cend())
                                                  { // we are adding a new term to 'r'..
                                                      r->l.vars.emplace(v, c * cc);
                                                      std::lock_guard<std::mutex> lock(t_mtxs[v]);
                                                      t_watches[v].emplace(r);
                                                  }
                                                  else
                                                  { // we are updating an existing term of 'r'..
                                                      assert(trm_it->first == v);
                                                      trm_it->second += c * cc;
                                                      if (trm_it->second == rational::ZERO)
                                                      { // the updated term's coefficient has become equal to zero, hence we can remove the term..
                                                          r->l.vars.erase(trm_it);
                                                          std::lock_guard<std::mutex> lock(t_mtxs[v]);
                                                          t_watches[v].erase(r);
                                                      }
                                                  }
                                              r->l.known_term += expr.known_term * cc;
                                          });
        // we wait for all the rows to be updated..
        sat.get_thread_pool().join();
#else
        { // 'r' is a row in which 'x_j' appears..
            rational cc = r->l.vars[x_j];
            r->l.vars.erase(x_j);
            for (const auto &[v, c] : std::map<const var, rational>(expr.vars))
                if (const auto trm_it = r->l.vars.find(v); trm_it == r->l.vars.cend())
                { // we are adding a new term to 'r'..
                    r->l.vars.emplace(v, c * cc);
                    t_watches[v].emplace(r);
                }
                else
                { // we are updating an existing term of 'r'..
                    assert(trm_it->first == v);
                    trm_it->second += c * cc;
                    if (trm_it->second == rational::ZERO)
                    { // the updated term's coefficient has become equal to zero, hence we can remove the term..
                        r->l.vars.erase(trm_it);
                        t_watches[v].erase(r);
                    }
                }
            r->l.known_term += expr.known_term * cc;
        }
#endif
        // we add a new row into the tableau..
        new_row(x_j, expr);
    }

    void lra_theory::new_row(const var &x, const lin &l) noexcept
    {
        row *r = new row(*this, x, l);
        tableau.emplace(x, r);
        for (const auto &[v, c] : l.vars)
            t_watches[v].emplace(r);
    }

    json lra_theory::to_json() const noexcept
    {
        json j_th;

        std::vector<json> j_vars;
        j_vars.reserve(vals.size());
        for (size_t i = 0; i < vals.size(); ++i)
        {
            json var;
            var->set("name", new string_val(std::to_string(i)));
            var->set("value", new string_val(to_string(value(i))));
            if (!is_negative_infinite(lb(i)))
                var->set("lb", new string_val(to_string(lb(i))));
            if (!is_positive_infinite(ub(i)))
                var->set("ub", new string_val(to_string(ub(i))));
            j_vars.push_back(var);
        }
        j_th->set("vars", new array_val(j_vars));

        std::vector<json> j_asrts;
        j_asrts.reserve(v_asrts.size());
        for (const auto &c_asrts : v_asrts)
        {
            std::vector<json> c_j_asrts;
            c_j_asrts.reserve(v_asrts.size());
            for (const auto &c_asrt : c_asrts.second)
                c_j_asrts.push_back(c_asrt->to_json());
            j_asrts.push_back(new array_val(c_j_asrts));
        }
        j_th->set("asrts", new array_val(j_asrts));

        std::vector<json> j_tabl;
        j_tabl.reserve(tableau.size());
        for (auto it = tableau.cbegin(); it != tableau.cend(); ++it)
            j_tabl.push_back(it->second->to_json());
        j_th->set("tableau", new array_val(j_tabl));

        return j_th;
    }
} // namespace smt