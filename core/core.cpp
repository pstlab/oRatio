#include "core.h"
#include "predicate.h"
#include "atom.h"
#include "field.h"
#include "method.h"
#include "core_parser.h"
#ifdef BUILD_LISTENERS
#include "core_listener.h"
#endif
#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cassert>

using namespace smt;

namespace ratio
{
    core::core() : scope(*this, *this), env(*this, context(this)), sat_cr(), lra_th(sat_cr), ov_th(sat_cr), idl_th(sat_cr), rdl_th(sat_cr) { new_types({new bool_type(*this), new int_type(*this), new real_type(*this), new tp_type(*this), new string_type(*this)}); }
    core::~core()
    {
        // we delete the predicates..
        for (const auto &p : predicates)
            delete p.second;

        // we delete the types..
        for (const auto &t : types)
            delete t.second;

        // we delete the methods..
        for (const auto &ms : methods)
            for (const auto &m : ms.second)
                delete m;

        // we delete the compilation units..
        for (const auto &cu : cus)
            delete cu;
    }

    void core::read(const std::string &script)
    {
        std::stringstream ss(script);
        riddle_parser prs(ss);
        ast::compilation_unit *cu = static_cast<ast::compilation_unit *>(prs.parse());
        cus.push_back(cu);

        cu->declare(*this);
        cu->refine(*this);
        context c_ctx(this);
        cu->execute(*this, c_ctx);
        FIRE_READ(script);

        if (!sat_cr.propagate())
            throw unsolvable_exception();
        FIRE_STATE_CHANGED();
    }

    void core::read(const std::vector<std::string> &files)
    {
        std::vector<ast::compilation_unit *> c_cus;
        for (const auto &f : files)
            if (std::ifstream ifs(f); ifs)
            {
                riddle_parser prs(ifs);
                ast::compilation_unit *cu = static_cast<ast::compilation_unit *>(prs.parse());
                ifs.close();
                c_cus.push_back(cu);
                cus.push_back(cu);
            }
            else
                throw std::invalid_argument("cannot find file '" + f + "'");

        for (const auto &cu : c_cus)
            cu->declare(*this);
        for (const auto &cu : c_cus)
            cu->refine(*this);
        context c_ctx(this);
        for (const auto &cu : c_cus)
            cu->execute(*this, c_ctx);
        FIRE_READ(files);

        if (!sat_cr.propagate())
            throw std::runtime_error("the input problem is inconsistent..");
        FIRE_STATE_CHANGED();
    }

    bool_expr core::new_bool() noexcept { return new bool_item(*this, lit(sat_cr.new_var())); }
    bool_expr core::new_bool(const bool &val) noexcept { return new bool_item(*this, val ? TRUE_lit : FALSE_lit); }

    arith_expr core::new_int() noexcept { return new arith_item(*this, *types.at(INT_KEYWORD), lin(lra_th.new_var(), rational::ONE)); }
    arith_expr core::new_int(const I &val) noexcept { return new arith_item(*this, *types.at(INT_KEYWORD), lin(rational(val))); }

    arith_expr core::new_real() noexcept { return new arith_item(*this, *types.at(REAL_KEYWORD), lin(lra_th.new_var(), rational::ONE)); }
    arith_expr core::new_real(const rational &val) noexcept { return new arith_item(*this, *types.at(REAL_KEYWORD), lin(val)); }

    arith_expr core::new_tp() noexcept { return new arith_item(*this, *types.at(TP_KEYWORD), lin(rdl_th.new_var(), rational::ONE)); }
    arith_expr core::new_tp(const rational &val) noexcept { return new arith_item(*this, *types.at(TP_KEYWORD), lin(val)); }

    string_expr core::new_string() noexcept { return new string_item(*this, ""); }
    string_expr core::new_string(const std::string &val) noexcept { return new string_item(*this, val); }

    const type &core::get_type(const std::vector<arith_expr> &exprs) const
    {
        if (std::all_of(exprs.begin(), exprs.end(), [](const arith_expr &aex) { return aex->get_type().get_name().compare(INT_KEYWORD) == 0; }))
            return *types.at(INT_KEYWORD);
        else if (std::all_of(exprs.begin(), exprs.end(), [](const arith_expr &aex) { return aex->get_type().get_name().compare(REAL_KEYWORD) == 0; }))
            return *types.at(REAL_KEYWORD);
        else if (std::all_of(exprs.begin(), exprs.end(), [this](const arith_expr &aex) { return aex->get_type().get_name().compare(TP_KEYWORD) == 0 || aex->l.vars.empty() || lra_th.lb(aex->l) == lra_th.ub(aex->l); }))
            return *types.at(TP_KEYWORD);
        else
            return *types.at(REAL_KEYWORD);
    }

    expr core::new_enum(const type &tp, const std::vector<item *> &allowed_vals)
    {
        assert(allowed_vals.size() > 1);
        assert(tp.get_name().compare(BOOL_KEYWORD) != 0);
        assert(tp.get_name().compare(INT_KEYWORD) != 0);
        assert(tp.get_name().compare(REAL_KEYWORD) != 0);
        assert(tp.get_name().compare(TP_KEYWORD) != 0);
        return new var_item(*this, tp, ov_th.new_var(std::vector<var_value *>(allowed_vals.begin(), allowed_vals.end())));
    }

    expr core::new_enum(const type &tp, const std::vector<lit> &lits, const std::vector<item *> &vals) noexcept
    {
        if (tp.get_name().compare(BOOL_KEYWORD) == 0)
        {
            bool_expr b = new_bool();
            bool nc;
            for (size_t i = 0; i < lits.size(); ++i)
            {
                nc = sat_cr.new_clause({!lits[i], sat_cr.new_eq(dynamic_cast<bool_item *>(vals[i])->l, b->l)});
                assert(nc);
            }
            return b;
        }
        else if (tp.get_name().compare(INT_KEYWORD) == 0)
        {
            auto min = inf_rational(rational::POSITIVE_INFINITY);
            auto max = inf_rational(rational::NEGATIVE_INFINITY);
            for (const auto &val : vals)
            {
                const auto bds = lra_th.bounds(dynamic_cast<arith_item *>(val)->l);
                if (min > bds.first)
                    min = bds.first;
                if (max < bds.second)
                    max = bds.second;
            }
            assert(min.get_infinitesimal() == rational::ZERO);
            assert(max.get_infinitesimal() == rational::ZERO);
            assert(min.get_rational().denominator() == 1);
            assert(max.get_rational().denominator() == 1);
            if (min == max) // we have a constant..
                return new_int(min.get_rational().numerator());
            else
            { // we need to create a new integer variable..
                arith_expr ie = new_int();
                bool nc;
                for (size_t i = 0; i < lits.size(); ++i)
                {
                    nc = sat_cr.new_clause({!lits[i], lra_th.new_eq(ie->l, dynamic_cast<arith_item *>(vals[i])->l)});
                    assert(nc);
                }
                // we impose some bounds which might help propagation..
                nc = sat_cr.new_clause({lra_th.new_geq(ie->l, lin(min.get_rational()))});
                assert(nc);
                nc = sat_cr.new_clause({lra_th.new_leq(ie->l, lin(max.get_rational()))});
                assert(nc);
                return ie;
            }
        }
        else if (tp.get_name().compare(REAL_KEYWORD) == 0)
        {
            auto min = inf_rational(rational::POSITIVE_INFINITY);
            auto max = inf_rational(rational::NEGATIVE_INFINITY);
            for (const auto &val : vals)
            {
                const auto bds = lra_th.bounds(dynamic_cast<arith_item *>(val)->l);
                if (min > bds.first)
                    min = bds.first;
                if (max < bds.second)
                    max = bds.second;
            }
            assert(min.get_infinitesimal() == rational::ZERO);
            assert(max.get_infinitesimal() == rational::ZERO);
            if (min == max) // we have a constant..
                return new_real(min.get_rational());
            else
            { // we need to create a new real variable..
                arith_expr re = new_real();
                bool nc;
                for (size_t i = 0; i < lits.size(); ++i)
                {
                    nc = sat_cr.new_clause({!lits[i], lra_th.new_eq(re->l, dynamic_cast<arith_item *>(vals[i])->l)});
                    assert(nc);
                }
                // we impose some bounds which might help propagation..
                nc = sat_cr.new_clause({lra_th.new_geq(re->l, lin(min.get_rational()))});
                assert(nc);
                nc = sat_cr.new_clause({lra_th.new_leq(re->l, lin(max.get_rational()))});
                assert(nc);
                return re;
            }
        }
        else if (tp.get_name().compare(TP_KEYWORD) == 0)
        {
            auto min = inf_rational(rational::POSITIVE_INFINITY);
            auto max = inf_rational(rational::NEGATIVE_INFINITY);
            for (const auto &val : vals)
            {
                const auto bds = rdl_th.bounds(dynamic_cast<arith_item *>(val)->l);
                if (min > bds.first)
                    min = bds.first;
                if (max < bds.second)
                    max = bds.second;
            }
            assert(min.get_infinitesimal() == rational::ZERO);
            assert(max.get_infinitesimal() == rational::ZERO);
            if (min == max) // we have a constant..
                return new_tp(min.get_rational());
            else
            { // we need to create a new time-point variable..
                arith_expr tp = new_tp();
                bool nc;
                for (size_t i = 0; i < lits.size(); ++i)
                {
                    nc = sat_cr.new_clause({!lits[i], rdl_th.new_eq(tp->l, dynamic_cast<arith_item *>(vals[i])->l)});
                    assert(nc);
                }
                // we impose some bounds which might help propagation..
                nc = sat_cr.new_clause({rdl_th.new_geq(tp->l, lin(min.get_rational()))});
                assert(nc);
                nc = sat_cr.new_clause({rdl_th.new_leq(tp->l, lin(max.get_rational()))});
                assert(nc);
                return tp;
            }
        }
        else
            return new var_item(*this, tp, ov_th.new_var(lits, std::vector<var_value *>(vals.begin(), vals.end())));
    }

    bool_expr core::negate(bool_expr var) noexcept { return new bool_item(*this, !var->l); }
    bool_expr core::eq(bool_expr left, bool_expr right) noexcept { return new bool_item(*this, sat_cr.new_eq(left->l, right->l)); }

    bool_expr core::conj(const std::vector<bool_expr> &exprs) noexcept
    {
        std::vector<lit> lits;
        for (const auto &bex : exprs)
            lits.push_back(bex->l);
        return new bool_item(*this, sat_cr.new_conj(lits));
    }

    bool_expr core::disj(const std::vector<bool_expr> &exprs) noexcept
    {
        std::vector<lit> lits;
        for (const auto &bex : exprs)
            lits.push_back(bex->l);
        return new bool_item(*this, sat_cr.new_disj(lits));
    }

    bool_expr core::exct_one(const std::vector<bool_expr> &exprs) noexcept
    {
        std::vector<lit> lits;
        for (const auto &bex : exprs)
            lits.push_back(bex->l);
        return new bool_item(*this, sat_cr.new_exct_one(lits));
    }

    arith_expr core::add(const std::vector<arith_expr> &exprs) noexcept
    {
        assert(exprs.size() > 1);
        lin l;
        for (const auto &aex : exprs)
            l += aex->l;
        return new arith_item(*this, get_type(exprs), l);
    }

    arith_expr core::sub(const std::vector<arith_expr> &exprs) noexcept
    {
        assert(exprs.size() > 1);
        lin l;
        for (std::vector<arith_expr>::const_iterator it = exprs.cbegin(); it != exprs.cend(); ++it)
            if (it == exprs.cbegin())
                l += (*it)->l;
            else
                l -= (*it)->l;
        return new arith_item(*this, get_type(exprs), l);
    }

    arith_expr core::mult(const std::vector<arith_expr> &exprs) noexcept
    {
        assert(exprs.size() > 1);
        arith_expr ae = *std::find_if(exprs.begin(), exprs.end(), [this](arith_expr ae) { return lra_th.lb(ae->l) == lra_th.ub(ae->l); });
        lin l = ae->l;
        for (const auto &aex : exprs)
            if (aex != ae)
            {
                assert(lra_th.lb(aex->l) == lra_th.ub(aex->l) && "non-linear expression..");
                assert(lra_th.value(aex->l).get_infinitesimal() == rational::ZERO);
                l *= lra_th.value(aex->l).get_rational();
            }
        return new arith_item(*this, get_type(exprs), l);
    }

    arith_expr core::div(const std::vector<arith_expr> &exprs) noexcept
    {
        assert(exprs.size() > 1);
        assert(std::all_of(++exprs.begin(), exprs.end(), [this](arith_expr ae) { return lra_th.lb(ae->l) == lra_th.ub(ae->l); }) && "non-linear expression..");
        assert(lra_th.value(exprs[1]->l).get_infinitesimal() == rational::ZERO);
        rational c = lra_th.value(exprs[1]->l).get_rational();
        for (size_t i = 2; i < exprs.size(); ++i)
        {
            assert(lra_th.value(exprs[i]->l).get_infinitesimal() == rational::ZERO);
            c *= lra_th.value(exprs[i]->l).get_rational();
        }
        return new arith_item(*this, get_type(exprs), exprs.at(0)->l / c);
    }

    arith_expr core::minus(arith_expr ex) noexcept { return new arith_item(*this, ex->get_type(), -ex->l); }

    bool_expr core::lt(arith_expr left, arith_expr right) noexcept
    {
        if (get_type({left, right}).get_name().compare(TP_KEYWORD) == 0)
            return new bool_item(*this, rdl_th.new_lt(left->l, right->l));
        else
            return new bool_item(*this, lra_th.new_lt(left->l, right->l));
    }
    bool_expr core::leq(arith_expr left, arith_expr right) noexcept
    {
        if (get_type({left, right}).get_name().compare(TP_KEYWORD) == 0)
            return new bool_item(*this, rdl_th.new_leq(left->l, right->l));
        else
            return new bool_item(*this, lra_th.new_leq(left->l, right->l));
    }
    bool_expr core::eq(arith_expr left, arith_expr right) noexcept
    {
        if (get_type({left, right}).get_name().compare(TP_KEYWORD) == 0)
            return new bool_item(*this, rdl_th.new_eq(left->l, right->l));
        else
            return new bool_item(*this, lra_th.new_eq(left->l, right->l));
    }
    bool_expr core::geq(arith_expr left, arith_expr right) noexcept
    {
        if (get_type({left, right}).get_name().compare(TP_KEYWORD) == 0)
            return new bool_item(*this, rdl_th.new_geq(left->l, right->l));
        else
            return new bool_item(*this, lra_th.new_geq(left->l, right->l));
    }
    bool_expr core::gt(arith_expr left, arith_expr right) noexcept
    {
        if (get_type({left, right}).get_name().compare(TP_KEYWORD) == 0)
            return new bool_item(*this, rdl_th.new_gt(left->l, right->l));
        else
            return new bool_item(*this, lra_th.new_gt(left->l, right->l));
    }

    bool_expr core::eq(expr left, expr right) noexcept { return new bool_item(*this, left->new_eq(*right)); }

    void core::assert_facts(const std::vector<lit> &facts)
    {
        for (const auto &f : facts)
            if (!sat_cr.new_clause({!ni, f}))
                throw unsolvable_exception();
    }

    void core::assert_facts(const std::vector<bool_expr> &facts)
    {
        for (const auto &f : facts)
            if (!sat_cr.new_clause({!ni, f->l}))
                throw unsolvable_exception();
    }

    void core::new_methods(const std::vector<const method *> &ms) noexcept
    {
        for (const auto &m : ms)
            methods[m->get_name()].push_back(m);
    }

    void core::new_types(const std::vector<type *> &ts) noexcept
    {
        for (const auto &t : ts)
            types.emplace(t->get_name(), t);
    }

    void core::new_predicates(const std::vector<predicate *> &ps) noexcept
    {
        for (const auto &p : ps)
            predicates.emplace(p->get_name(), p);
    }

    const field &core::get_field(const std::string &name) const
    {
        if (const auto at_f = fields.find(name); at_f != fields.end())
            return *at_f->second;

        // not found
        throw std::out_of_range(name);
    }

    const method &core::get_method(const std::string &name, const std::vector<const type *> &ts) const
    {
        if (const auto at_m = methods.find(name); at_m != methods.end())
        {
            bool found = false;
            for (const auto &mthd : at_m->second)
                if (mthd->args.size() == ts.size())
                {
                    found = true;
                    for (size_t i = 0; i < ts.size(); ++i)
                        if (!mthd->args[i]->get_type().is_assignable_from(*ts[i]))
                        {
                            found = false;
                            break;
                        }
                    if (found)
                        return *mthd;
                }
        }

        // not found
        throw std::out_of_range(name);
    }

    type &core::get_type(const std::string &name) const
    {
        if (const auto at_tp = types.find(name); at_tp != types.end())
            return *at_tp->second;

        // not found
        throw std::out_of_range(name);
    }

    predicate &core::get_predicate(const std::string &name) const
    {
        if (const auto at_p = predicates.find(name); at_p != predicates.end())
            return *at_p->second;

        // not found
        throw std::out_of_range(name);
    }

    expr core::get(const std::string &name) const
    {
        if (const auto at_xpr = exprs.find(name); at_xpr != exprs.end())
            return at_xpr->second;

        throw std::out_of_range(name);
    }

    lbool core::bool_value(const bool_expr &x) const noexcept { return sat_cr.value(x->l); }
    std::pair<inf_rational, inf_rational> core::arith_bounds(const arith_expr &x) const noexcept
    {
        if (x->get_type().get_name().compare(TP_KEYWORD) == 0)
            return rdl_th.bounds(x->l);
        else
            return lra_th.bounds(x->l);
    }
    inf_rational core::arith_value(const arith_expr &x) const noexcept
    {
        if (x->get_type().get_name().compare(TP_KEYWORD) == 0)
            return rdl_th.bounds(x->l).first;
        else
            return lra_th.value(x->l);
    }
    std::unordered_set<const var_value *> core::enum_value(const var_expr &x) const noexcept { return ov_th.value(x->ev); }

    json core::to_json() const noexcept
    {
        std::set<item *> all_items;
        std::set<atom *> all_atoms;
        for (const auto &p : predicates)
            for (const auto &a : p.second->get_instances())
                all_atoms.insert(static_cast<atom *>(&*a));
        std::queue<type *> q;
        for (const auto &t : types)
            if (!t.second->is_primitive())
                q.push(t.second);
        while (!q.empty())
        {
            for (const auto &i : q.front()->get_instances())
                all_items.insert(&*i);
            for (const auto &p : q.front()->get_predicates())
                for (const auto &a : p.second->get_instances())
                    all_atoms.insert(static_cast<atom *>(&*a));
            for (const auto &t : q.front()->get_types())
                q.push(t.second);
            q.pop();
        }

        json j_core;

        if (!all_items.empty())
        {
            std::vector<json> j_itms;
            for (const auto &itm : all_items)
                j_itms.push_back(itm->to_json());
            j_core->set("items", new array_val(j_itms));
        }

        if (!all_atoms.empty())
        {
            std::vector<json> j_atms;
            for (const auto &atm : all_atoms)
                j_atms.push_back(atm->to_json());
            j_core->set("atoms", new array_val(j_atms));
        }

        if (!env::exprs.empty())
            j_core->set("exprs", env::to_json());
        return j_core;
    }

    std::ostream &operator<<(std::ostream &os, const core &cr)
    {
        cr.to_json().to_json(os);
        return os;
    }

#ifdef BUILD_LISTENERS
    void core::fire_log(const std::string msg) const noexcept
    {
        for (const auto &l : listeners)
            l->log(msg);
    }
    void core::fire_read(const std::string &script) const noexcept
    {
        for (const auto &l : listeners)
            l->read(script);
    }
    void core::fire_read(const std::vector<std::string> &files) const noexcept
    {
        for (const auto &l : listeners)
            l->read(files);
    }
    void core::fire_state_changed() const noexcept
    {
        for (const auto &l : listeners)
            l->state_changed();
    }
    void core::fire_started_solving() const noexcept
    {
        for (const auto &l : listeners)
            l->started_solving();
    }
    void core::fire_solution_found() const noexcept
    {
        for (const auto &l : listeners)
            l->solution_found();
    }
    void core::fire_inconsistent_problem() const noexcept
    {
        for (const auto &l : listeners)
            l->inconsistent_problem();
    }
#endif
} // namespace ratio