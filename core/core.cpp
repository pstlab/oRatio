#include "core.h"
#include "predicate.h"
#include "atom.h"
#include "field.h"
#include "method.h"
#include "riddle_parser.h"
#ifdef BUILD_GUI
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
    std::stringstream ss(script);
    riddle_parser prs(ss);
    ast::compilation_unit *cu = static_cast<ast::compilation_unit *>(prs.parse());
    cus.push_back(cu);

    cu->declare(*this);
    cu->refine(*this);
    context c_ctx(this);
    cu->execute(*this, c_ctx);
#ifdef BUILD_GUI
    fire_read(script);
#endif

    if (!sat_cr.check())
        throw unsolvable_exception();
#ifdef BUILD_GUI
    fire_state_changed();
#endif
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
            throw unsolvable_exception();

    for (const auto &cu : c_cus)
        cu->declare(*this);
    for (const auto &cu : c_cus)
        cu->refine(*this);
    context c_ctx(this);
    for (const auto &cu : c_cus)
        cu->execute(*this, c_ctx);
#ifdef BUILD_GUI
    fire_read(files);
#endif

    if (!sat_cr.check())
        throw std::runtime_error("the input problem is inconsistent..");
#ifdef BUILD_GUI
    fire_state_changed();
#endif
}

bool_expr core::new_bool() { return new bool_item(*this, sat_cr.new_var()); }
bool_expr core::new_bool(const bool &val) { return new bool_item(*this, val); }

arith_expr core::new_int() { return new arith_item(*this, *types.at(INT_KEYWORD), lin(lra_th.new_var(), 1)); }
arith_expr core::new_int(const I &val) { return new arith_item(*this, *types.at(INT_KEYWORD), lin(val)); }

arith_expr core::new_real() { return new arith_item(*this, *types.at(REAL_KEYWORD), lin(lra_th.new_var(), 1)); }
arith_expr core::new_real(const rational &val) { return new arith_item(*this, *types.at(REAL_KEYWORD), lin(val)); }

string_expr core::new_string() { return new string_item(*this, ""); }
string_expr core::new_string(const std::string &val) { return new string_item(*this, val); }

expr core::new_enum(const type &tp, const std::vector<item *> &allowed_vals)
{
    assert(allowed_vals.size() > 1);
    assert(tp.get_name().compare(BOOL_KEYWORD) != 0);
    assert(tp.get_name().compare(INT_KEYWORD) != 0);
    assert(tp.get_name().compare(REAL_KEYWORD) != 0);
    return new var_item(*this, tp, ov_th.new_var(std::vector<var_value *>(allowed_vals.begin(), allowed_vals.end())));
}

expr core::new_enum(const type &tp, const std::vector<var> &vars, const std::vector<item *> &vals)
{
    if (tp.get_name().compare(BOOL_KEYWORD) == 0)
    {
        bool_expr b = new_bool();
        bool nc;
        for (size_t i = 0; i < vars.size(); ++i)
        {
            nc = sat_cr.new_clause({lit(vars[i], false), sat_cr.new_eq(dynamic_cast<bool_item *>(vals[i])->l, b->l)});
            assert(nc);
        }
        return b;
    }
    else if (tp.get_name().compare(INT_KEYWORD) == 0)
    {
        arith_expr ie = new_int();
        bool nc;
        for (size_t i = 0; i < vars.size(); ++i)
        {
            nc = sat_cr.new_clause({lit(vars[i], false), sat_cr.new_conj({lra_th.new_leq(ie->l, dynamic_cast<arith_item *>(vals[i])->l), lra_th.new_geq(ie->l, dynamic_cast<arith_item *>(vals[i])->l)})});
            assert(nc);
        }
        return ie;
    }
    else if (tp.get_name().compare(REAL_KEYWORD) == 0)
    {
        arith_expr re = new_real();
        bool nc;
        for (size_t i = 0; i < vars.size(); ++i)
        {
            nc = sat_cr.new_clause({lit(vars[i], false), sat_cr.new_conj({lra_th.new_leq(re->l, dynamic_cast<arith_item *>(vals[i])->l), lra_th.new_geq(re->l, dynamic_cast<arith_item *>(vals[i])->l)})});
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
    arith_expr ae = *std::find_if(exprs.begin(), exprs.end(), [this](arith_expr ae) { return lra_th.lb(ae->l) == lra_th.ub(ae->l); });
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
    assert(std::all_of(++exprs.begin(), exprs.end(), [this](arith_expr ae) { return lra_th.lb(ae->l) == lra_th.ub(ae->l); }) && "non-linear expression..");
    assert(lra_th.value(exprs[1]->l).get_infinitesimal() == rational::ZERO);
    rational c = lra_th.value(exprs[1]->l).get_rational();
    for (size_t i = 2; i < exprs.size(); ++i)
    {
        assert(lra_th.value(exprs[i]->l).get_infinitesimal() == rational::ZERO);
        c *= lra_th.value(exprs[i]->l).get_rational();
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
        if (!sat_cr.new_clause({lit(ni, false), f}))
            throw unsolvable_exception();
}

void core::new_methods(const std::vector<const method *> &ms)
{
    for (const auto &m : ms)
        methods[m->get_name()].push_back(m);
}

void core::new_types(const std::vector<type *> &ts)
{
    for (const auto &t : ts)
        types.emplace(t->get_name(), t);
}

void core::new_predicates(const std::vector<predicate *> &ps)
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
        iss += "{ \"name\" : \"" + is_it->first + "\", \"type\" : \"";

        std::string i_type = is_it->second->get_type().get_name();
        const type *t = &is_it->second->get_type();
        while (const type *sc = dynamic_cast<const type *>((&t->get_scope())))
        {
            i_type.insert(0, sc->get_name() + ":");
            t = sc;
        }
        iss += i_type + "\", \"value\" : ";

        if (bool_item *bi = dynamic_cast<bool_item *>(&*is_it->second)) // the expression represents a primitive bool type..
        {
            std::string sign_s = bi->l.get_sign() ? "b" : "!b";
            iss += "{ \"lit\" : \"" + sign_s + std::to_string(bi->l.get_var()) + "\", \"val\" : ";
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
        else if (arith_item *ai = dynamic_cast<arith_item *>(&*is_it->second)) // the expression represents a primitive arithmetic type..
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
        else if (var_item *ei = dynamic_cast<var_item *>(&*is_it->second)) // the expression represents an enum type..
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
        else if (string_item *si = dynamic_cast<string_item *>(&*is_it->second)) // the expression represents a primitive string type..
            iss += "\"" + si->get_value() + "\"";
        else // the expression represents an item..
            iss += "\"" + std::to_string(reinterpret_cast<uintptr_t>(&*is_it->second)) + "\"";
        iss += " }";
    }
    return iss;
}

std::string core::to_string(const item *const i) const noexcept
{
    std::string is;
    is += "{ \"id\" : \"" + std::to_string(reinterpret_cast<uintptr_t>(i)) + "\", \"type\" : \"";

    std::string i_type = i->get_type().get_name();
    const type *t = &i->get_type();
    while (const type *sc = dynamic_cast<const type *>((&t->get_scope())))
    {
        i_type.insert(0, sc->get_name() + ":");
        t = sc;
    }
    is += i_type + "\"";

    if (std::map<std::string, expr> c_is = i->get_exprs(); !c_is.empty())
        is += ", \"exprs\" : [ " + to_string(c_is) + " ]";
    is += "}";
    return is;
}

std::string core::to_string(const atom *const a) const noexcept
{
    std::string as;
    as += "{ \"id\" : \"" + std::to_string(reinterpret_cast<uintptr_t>(a)) + "\", \"predicate\" : \"";

    std::string a_type = a->get_type().get_name();
    const type *t = &a->get_type();
    while (const type *sc = dynamic_cast<const type *>((&t->get_scope())))
    {
        a_type.insert(0, sc->get_name() + ":");
        t = sc;
    }
    as += a_type + "\", \"sigma\" : " + std::to_string(a->get_sigma()) + ", \"state\" : ";

    switch (sat_cr.value(a->get_sigma()))
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

    if (std::map<std::string, expr> is = a->get_exprs(); !is.empty())
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
        if (!t.second->is_primitive())
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
    cr += "\"exprs\" : [" + to_string(get_exprs()) + "] }";
    return cr;
}

#ifdef BUILD_GUI
void core::fire_log(const std::string msg) const
{
    for (const auto &l : listeners)
        l->log(msg);
}
void core::fire_read(const std::string &script) const
{
    for (const auto &l : listeners)
        l->read(script);
}
void core::fire_read(const std::vector<std::string> &files) const
{
    for (const auto &l : listeners)
        l->read(files);
}
void core::fire_state_changed() const
{
    for (const auto &l : listeners)
        l->state_changed();
}
#endif
} // namespace ratio
