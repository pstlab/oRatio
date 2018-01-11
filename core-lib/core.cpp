#include "core.h"
#include "predicate.h"
#include "atom.h"
#include "method.h"
#include "field.h"
#include "parser.h"
#include "compilation_unit.h"
#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cassert>

using namespace smt;

namespace ratio
{

core::core() : scope(*this, *this), env(*this, this), sat_cr(), lra_th(sat_cr), ov_th(sat_cr) { new_types({new bool_type(*this), new int_type(*this), new real_type(*this), new string_type(*this)}); }

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
#ifdef STATISTICS
    auto start_parsing = std::chrono::steady_clock::now();
#endif

    std::stringstream ss(script);
    parser prs(ss);
    ast::compilation_unit *cu = prs.parse();
    cus.push_back(cu);

    cu->declare(*this);
    cu->refine(*this);
    context c_ctx(this);
    cu->execute(*this, c_ctx);

#ifdef STATISTICS
    auto end_parsing = std::chrono::steady_clock::now();
    parsing_time += end_parsing - start_parsing;
#endif

    if (!sat_cr.check())
        throw unsolvable_exception("the input problem is inconsistent");
}

void core::read(const std::vector<std::string> &files)
{
#ifdef STATISTICS
    auto start_parsing = std::chrono::steady_clock::now();
#endif

    std::vector<ast::compilation_unit *> c_cus;
    for (const auto &f : files)
    {
        std::ifstream ifs(f);
        if (ifs)
        {
            parser prs(ifs);
            ast::compilation_unit *cu = prs.parse();
            ifs.close();
            c_cus.push_back(cu);
            cus.push_back(cu);
        }
        else
            throw std::invalid_argument("file not found: " + f);
    }

    for (const auto &cu : c_cus)
        cu->declare(*this);
    for (const auto &cu : c_cus)
        cu->refine(*this);
    context c_ctx(this);
    for (const auto &cu : c_cus)
        cu->execute(*this, c_ctx);

#ifdef STATISTICS
    auto end_parsing = std::chrono::steady_clock::now();
    parsing_time += end_parsing - start_parsing;
#endif

    if (!sat_cr.check())
        throw unsolvable_exception("the input problem is inconsistent");
}

bool_expr core::new_bool() { return new bool_item(*this, sat_cr.new_var()); }
bool_expr core::new_bool(const bool &val) { return new bool_item(*this, val); }

arith_expr core::new_int() { return new arith_item(*this, *types.at(INT_KEYWORD), lin(lra_th.new_var(), 1)); }
arith_expr core::new_int(const I &val) { return new arith_item(*this, *types.at(INT_KEYWORD), lin(val)); }

arith_expr core::new_real() { return new arith_item(*this, *types.at(REAL_KEYWORD), lin(lra_th.new_var(), 1)); }
arith_expr core::new_real(const rational &val) { return new arith_item(*this, *types.at(REAL_KEYWORD), lin(val)); }

string_expr core::new_string() { return new string_item(*this, ""); }
string_expr core::new_string(const std::string &val) { return new string_item(*this, val); }

expr core::new_enum(const type &tp, const std::unordered_set<item *> &allowed_vals)
{
    assert(allowed_vals.size() > 1);
    assert(tp.name.compare(BOOL_KEYWORD) != 0);
    assert(tp.name.compare(INT_KEYWORD) != 0);
    assert(tp.name.compare(REAL_KEYWORD) != 0);
    return new var_item(*this, tp, ov_th.new_var(std::unordered_set<var_value *>(allowed_vals.begin(), allowed_vals.end())));
}

expr core::new_enum(const type &tp, const std::vector<var> &vars, const std::vector<item *> &vals)
{
    if (tp.name.compare(BOOL_KEYWORD) == 0)
    {
        bool_expr b = new_bool();
        bool nc;
        for (size_t i = 0; i < vars.size(); ++i)
        {
            nc = sat_cr.new_clause({lit(vars.at(i), false), sat_cr.new_eq(dynamic_cast<bool_item *>(vals.at(i))->l, b->l)});
            assert(nc);
        }
        return b;
    }
    else if (tp.name.compare(INT_KEYWORD) == 0)
    {
        arith_expr ie = new_int();
        bool nc;
        for (size_t i = 0; i < vars.size(); ++i)
        {
            nc = sat_cr.new_clause({lit(vars.at(i), false), sat_cr.new_conj({lra_th.new_leq(ie->l, dynamic_cast<arith_item *>(vals.at(i))->l), lra_th.new_geq(ie->l, dynamic_cast<arith_item *>(vals.at(i))->l)})});
            assert(nc);
        }
        return ie;
    }
    else if (tp.name.compare(REAL_KEYWORD) == 0)
    {
        arith_expr re = new_real();
        bool nc;
        for (size_t i = 0; i < vars.size(); ++i)
        {
            nc = sat_cr.new_clause({lit(vars.at(i), false), sat_cr.new_conj({lra_th.new_leq(re->l, dynamic_cast<arith_item *>(vals.at(i))->l), lra_th.new_geq(re->l, dynamic_cast<arith_item *>(vals.at(i))->l)})});
            assert(nc);
        }
        return re;
    }
    else
        return new var_item(*this, tp, ov_th.new_var(vars, std::vector<var_value *>(vals.begin(), vals.end())));
}

bool_expr core::negate(bool_expr var) { return new bool_item(*this, !var->l); }
bool_expr core::eq(bool_expr left, bool_expr right) { return new bool_item(*this, sat_cr.new_eq(left->l, right->l)); }

bool_expr core::conj(const std::vector<bool_expr> &exprs)
{
    std::vector<lit> lits;
    for (const auto &bex : exprs)
        lits.push_back(bex->l);
    return new bool_item(*this, sat_cr.new_conj(lits));
}

bool_expr core::disj(const std::vector<bool_expr> &exprs)
{
    std::vector<lit> lits;
    for (const auto &bex : exprs)
        lits.push_back(bex->l);
    return new bool_item(*this, sat_cr.new_disj(lits));
}

bool_expr core::exct_one(const std::vector<bool_expr> &exprs)
{
    std::vector<lit> lits;
    for (const auto &bex : exprs)
        lits.push_back(bex->l);
    return new bool_item(*this, sat_cr.new_exct_one(lits));
}

arith_expr core::add(const std::vector<arith_expr> &exprs)
{
    assert(exprs.size() > 1);
    lin l;
    for (const auto &aex : exprs)
        l += aex->l;
    return new arith_item(*this, *types.at(REAL_KEYWORD), l);
}

arith_expr core::sub(const std::vector<arith_expr> &exprs)
{
    assert(exprs.size() > 1);
    lin l;
    for (std::vector<arith_expr>::const_iterator it = exprs.cbegin(); it != exprs.cend(); ++it)
        if (it == exprs.cbegin())
            l += (*it)->l;
        else
            l -= (*it)->l;
    return new arith_item(*this, *types.at(REAL_KEYWORD), l);
}

arith_expr core::mult(const std::vector<arith_expr> &exprs)
{
    assert(exprs.size() > 1);
    arith_expr ae = *std::find_if(exprs.begin(), exprs.end(), [&](arith_expr ae) { return lra_th.lb(ae->l) == lra_th.ub(ae->l); });
    lin l = ae->l;
    for (const auto &aex : exprs)
        if (aex != ae)
        {
            assert(lra_th.lb(aex->l) == lra_th.ub(aex->l) && "non-linear expression..");
            assert(lra_th.value(aex->l).get_infinitesimal() == rational::ZERO);
            l *= lra_th.value(aex->l).get_rational();
        }
    return new arith_item(*this, *types.at(REAL_KEYWORD), l);
}

arith_expr core::div(const std::vector<arith_expr> &exprs)
{
    assert(exprs.size() > 1);
    assert(std::all_of(++exprs.begin(), exprs.end(), [&](arith_expr ae) { return lra_th.lb(ae->l) == lra_th.ub(ae->l); }) && "non-linear expression..");
    assert(lra_th.value(exprs.at(1)->l).get_infinitesimal() == rational::ZERO);
    rational c = lra_th.value(exprs.at(1)->l).get_rational();
    for (size_t i = 2; i < exprs.size(); i++)
    {
        assert(lra_th.value(exprs.at(i)->l).get_infinitesimal() == rational::ZERO);
        c *= lra_th.value(exprs.at(i)->l).get_rational();
    }
    return new arith_item(*this, *types.at(REAL_KEYWORD), exprs.at(0)->l / c);
}

arith_expr core::minus(arith_expr ex) { return new arith_item(*this, *types.at(REAL_KEYWORD), -ex->l); }

bool_expr core::lt(arith_expr left, arith_expr right) { return new bool_item(*this, lra_th.new_lt(left->l, right->l)); }
bool_expr core::leq(arith_expr left, arith_expr right) { return new bool_item(*this, lra_th.new_leq(left->l, right->l)); }
bool_expr core::eq(arith_expr left, arith_expr right) { return new bool_item(*this, sat_cr.new_conj({lra_th.new_leq(left->l, right->l), lra_th.new_geq(left->l, right->l)})); }
bool_expr core::geq(arith_expr left, arith_expr right) { return new bool_item(*this, lra_th.new_geq(left->l, right->l)); }
bool_expr core::gt(arith_expr left, arith_expr right) { return new bool_item(*this, lra_th.new_gt(left->l, right->l)); }

bool_expr core::eq(expr left, expr right) { return new bool_item(*this, left->eq(*right)); }

void core::assert_facts(const std::vector<lit> &facts)
{
    for (const auto &f : facts)
        if (!sat_cr.new_clause({lit(ctr_var, false), f}))
            throw unsolvable_exception();
}

field &core::get_field(const std::string &name) const
{
    const auto at_f = fields.find(name);
    if (at_f != fields.end())
        return *at_f->second;

    // not found
    throw std::out_of_range(name);
}

void core::new_methods(const std::vector<const method *> &ms)
{
    for (const auto &m : ms)
        methods[m->name].push_back(m);
}

const method &core::get_method(const std::string &name, const std::vector<const type *> &ts) const
{
    const auto at_m = methods.find(name);
    if (at_m != methods.end())
    {
        bool found = false;
        for (const auto &mthd : at_m->second)
            if (mthd->args.size() == ts.size())
            {
                found = true;
                for (size_t i = 0; i < ts.size(); i++)
                    if (!mthd->args.at(i)->tp.is_assignable_from(*ts.at(i)))
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

void core::new_predicates(const std::vector<predicate *> &ps)
{
    for (const auto &p : ps)
        predicates.insert({p->name, p});
}

predicate &core::get_predicate(const std::string &name) const
{
    const auto at_p = predicates.find(name);
    if (at_p != predicates.end())
        return *at_p->second;

    // not found
    throw std::out_of_range(name);
}

void core::new_types(const std::vector<type *> &ts)
{
    for (const auto &t : ts)
        types.insert({t->name, t});
}

type &core::get_type(const std::string &name) const
{
    const auto at_tp = types.find(name);
    if (at_tp != types.end())
        return *at_tp->second;

    // not found
    throw std::out_of_range(name);
}

expr core::get(const std::string &name) const
{
    const auto at_itm = items.find(name);
    if (at_itm != items.end())
        return at_itm->second;

    throw std::out_of_range(name);
}

lbool core::bool_value(const bool_expr &x) const noexcept { return sat_cr.value(x->l); }
inf_rational core::arith_lb(const arith_expr &x) const noexcept { return lra_th.lb(x->l); }
inf_rational core::arith_ub(const arith_expr &x) const noexcept { return lra_th.ub(x->l); }
inf_rational core::arith_value(const arith_expr &x) const noexcept { return lra_th.value(x->l); }
std::unordered_set<var_value *> core::enum_value(const var_expr &x) const noexcept { return ov_th.value(x->ev); }

std::string core::to_string(const std::map<std::string, expr> &c_items) const noexcept
{
    std::string iss;
    for (std::map<std::string, expr>::const_iterator is_it = c_items.cbegin(); is_it != c_items.cend(); ++is_it)
    {
        if (is_it != c_items.cbegin())
            iss += ", ";
        iss += "{ \"name\" : \"" + is_it->first + "\", \"type\" : \"" + is_it->second->tp.name + "\", \"value\" : ";
        if (bool_item *bi = dynamic_cast<bool_item *>(&*is_it->second))
        {
            std::string sign_s = bi->l.sign ? "b" : "!b";
            iss += "{ \"lit\" : \"" + sign_s + std::to_string(bi->l.v) + "\", \"val\" : ";
            switch (sat_cr.value(bi->l))
            {
            case True:
                iss += "\"True\"";
                break;
            case False:
                iss += "\"False\"";
                break;
            case Undefined:
                iss += "\"Undefined\"";
                break;
            }
            iss += " }";
        }
        else if (arith_item *ai = dynamic_cast<arith_item *>(&*is_it->second))
        {
            const auto val = lra_th.value(ai->l);
            iss += "{ \"lin\" : \"" + ai->l.to_string() + "\", \"val\" : ";
            iss += "{ \"num\" : " + std::to_string(val.get_rational().numerator()) + ", \"den\" : " + std::to_string(val.get_rational().denominator());
            if (val.get_infinitesimal() != rational::ZERO)
                iss += ", \"inf\" : { \"num\" : " + std::to_string(val.get_infinitesimal().numerator()) + ", \"den\" : " + std::to_string(val.get_infinitesimal().denominator()) + " }";
            iss += " }";
            const auto lb = lra_th.lb(ai->l);
            if (!lb.is_negative_infinite())
            {
                iss += ", \"lb\" : { \"num\" : " + std::to_string(lb.get_rational().numerator()) + ", \"den\" : " + std::to_string(lb.get_rational().denominator());
                if (val.get_infinitesimal() != rational::ZERO)
                    iss += ", \"inf\" : { \"num\" : " + std::to_string(lb.get_infinitesimal().numerator()) + ", \"den\" : " + std::to_string(lb.get_infinitesimal().denominator()) + " }";
                iss += " }";
            }
            const auto ub = lra_th.ub(ai->l);
            if (!ub.is_positive_infinite())
            {
                iss += ", \"ub\" : { \"num\" : " + std::to_string(ub.get_rational().numerator()) + ", \"den\" : " + std::to_string(ub.get_rational().denominator());
                if (val.get_infinitesimal() != rational::ZERO)
                    iss += ", \"inf\" : { \"num\" : " + std::to_string(ub.get_infinitesimal().numerator()) + ", \"den\" : " + std::to_string(ub.get_infinitesimal().denominator()) + " }";
                iss += " }";
            }
            iss += " }";
        }
        else if (var_item *ei = dynamic_cast<var_item *>(&*is_it->second))
        {
            iss += "{ \"var\" : \"e" + std::to_string(ei->ev) + "\", \"vals\" : [ ";
            std::unordered_set<var_value *> vals = ov_th.value(ei->ev);
            for (std::unordered_set<var_value *>::iterator vals_it = vals.begin(); vals_it != vals.end(); ++vals_it)
            {
                if (vals_it != vals.begin())
                    iss += ", ";
                iss += "\"" + std::to_string(reinterpret_cast<uintptr_t>(static_cast<item *>(*vals_it))) + "\"";
            }
            iss += " ] }";
        }
        else
            iss += "\"" + std::to_string(reinterpret_cast<uintptr_t>(&*is_it->second)) + "\"";
        iss += " }";
    }
    return iss;
}

std::string core::to_string(const item *const i) const noexcept
{
    std::string is;
    is += "{ \"id\" : \"" + std::to_string(reinterpret_cast<uintptr_t>(i)) + "\", \"type\" : \"" + i->tp.name + "\"";
    std::map<std::string, expr> c_is = i->get_items();
    if (!c_is.empty())
        is += ", \"items\" : [ " + to_string(c_is) + " ]";
    is += "}";
    return is;
}

std::string core::to_string(const atom *const a) const noexcept
{
    std::string as;
    as += "{ \"id\" : \"" + std::to_string(reinterpret_cast<uintptr_t>(a)) + "\", \"predicate\" : \"" + a->tp.name + "\", \"state\" : ";
    switch (sat_cr.value(a->sigma))
    {
    case True:
        as += "\"Active\"";
        break;
    case False:
        as += "\"Unified\"";
        break;
    case Undefined:
        as += "\"Inactive\"";
        break;
    }
    std::map<std::string, expr> is = a->get_items();
    if (!is.empty())
        as += ", \"pars\" : [ " + to_string(is) + " ]";
    as += "}";
    return as;
}

std::string core::to_string() const noexcept
{
    std::set<item *> all_items;
    std::set<atom *> all_atoms;
    for (const auto &p : get_predicates())
        for (const auto &a : p.second->get_instances())
            all_atoms.insert(static_cast<atom *>(&*a));
    std::queue<type *> q;
    for (const auto &t : get_types())
        if (!t.second->primitive)
            q.push(t.second);
    while (!q.empty())
    {
        for (const auto &i : q.front()->get_instances())
            all_items.insert(&*i);
        for (const auto &p : q.front()->get_predicates())
            for (const auto &a : p.second->get_instances())
                all_atoms.insert(static_cast<atom *>(&*a));
        q.pop();
    }

    std::string cr;
    cr += "{ ";
    if (!all_items.empty())
    {
        cr += "\"items\" : [";
        for (std::set<item *>::iterator is_it = all_items.begin(); is_it != all_items.end(); ++is_it)
        {
            if (is_it != all_items.begin())
                cr += ", ";
            cr += to_string(*is_it);
        }
        cr += "]";
    }
    if (!all_atoms.empty())
    {
        if (!all_items.empty())
            cr += ", ";
        cr += "\"atoms\" : [";
        for (std::set<atom *>::iterator as_it = all_atoms.begin(); as_it != all_atoms.end(); ++as_it)
        {
            if (as_it != all_atoms.begin())
                cr += ", ";
            cr += to_string(*as_it);
        }
        cr += "]";
    }
    if (!all_items.empty() || !all_atoms.empty())
        cr += ", ";
    cr += "\"refs\" : [" + to_string(get_items()) + "] }";
    return cr;
}
}