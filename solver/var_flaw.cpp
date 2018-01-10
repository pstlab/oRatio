#include "var_flaw.h"
#include "solver.h"

namespace ratio
{

inline const std::vector<resolver *> get_cause(resolver *const cause)
{
    if (cause)
        return {cause};
    else
        return {};
}

var_flaw::var_flaw(solver &slv, resolver *const cause, var_item &v_itm) : flaw(slv, get_cause(cause), true, true), v_itm(v_itm) {}
var_flaw::~var_flaw() {}

void var_flaw::compute_resolvers()
{
    std::unordered_set<smt::var_value *> vals = slv.ov_th.value(v_itm.ev);
    for (const auto &v : vals)
        add_resolver(*new choose_value(slv, smt::rational(1, vals.size()), *this, *v));
}

var_flaw::choose_value::choose_value(solver &slv, smt::rational cst, var_flaw &enm_flaw, smt::var_value &val) : resolver(slv, slv.ov_th.allows(enm_flaw.v_itm.ev, val), cst, enm_flaw), v(enm_flaw.v_itm.ev), val(val) {}
var_flaw::choose_value::~choose_value() {}
void var_flaw::choose_value::apply() {}
}