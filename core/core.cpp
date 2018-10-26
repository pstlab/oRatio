#include "core.h"
#include "predicate.h"
#include "atom.h"
#include <cassert>

using namespace smt;

namespace ratio
{

core::core() : scope(*this, *this), env(*this, this), sat_cr(), lra_th(sat_cr), ov_th(sat_cr) {}

core::~core() {}

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
    assert(tp.get_name().compare(BOOL_KEYWORD) != 0);
    assert(tp.get_name().compare(INT_KEYWORD) != 0);
    assert(tp.get_name().compare(REAL_KEYWORD) != 0);
    return new var_item(*this, tp, ov_th.new_var(std::unordered_set<var_value *>(allowed_vals.begin(), allowed_vals.end())));
}

expr core::new_enum(const type &tp, const std::vector<var> &vars, const std::vector<item *> &vals)
{
    if (tp.get_name().compare(BOOL_KEYWORD) == 0)
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
    else if (tp.get_name().compare(INT_KEYWORD) == 0)
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
    else if (tp.get_name().compare(REAL_KEYWORD) == 0)
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

field &core::get_field(const std::string &name) const
{
    const auto at_f = fields.find(name);
    if (at_f != fields.end())
        return *at_f->second;

    // not found
    throw std::out_of_range(name);
}

expr core::get(const std::string &name) const
{
    const auto at_xpr = exprs.find(name);
    if (at_xpr != exprs.end())
        return at_xpr->second;

    throw std::out_of_range(name);
}
} // namespace ratio