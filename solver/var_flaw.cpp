#include "var_flaw.h"
#include "solver.h"
#include "item.h"

namespace ratio
{

inline const std::vector<resolver *> get_cause(resolver *const cause)
{
    if (cause)
        return {cause};
    else
        return {};
}

var_flaw::var_flaw(graph &gr, resolver *const cause, var_item &v_itm) : flaw(gr, get_cause(cause), get_cause(cause), true), v_itm(v_itm) {}
var_flaw::~var_flaw() {}

#ifdef BUILD_GUI
std::string var_flaw::get_label() const
{
    return "φ" + std::to_string(get_phi()) + " enum";
}
#endif

void var_flaw::compute_resolvers()
{
    std::unordered_set<smt::var_value *> vals = get_graph().get_solver().get_ov_theory().value(v_itm.ev);
    for (const auto &v : vals)
        add_resolver(*new choose_value(get_graph(), smt::rational(1, static_cast<smt::I>(vals.size())), *this, *v));
}

var_flaw::choose_value::choose_value(graph &gr, smt::rational cst, var_flaw &enm_flaw, smt::var_value &val) : resolver(gr, gr.get_solver().get_ov_theory().allows(enm_flaw.v_itm.ev, val), cst, enm_flaw), v(enm_flaw.v_itm.ev), val(val) {}
var_flaw::choose_value::~choose_value() {}

#ifdef BUILD_GUI
std::string var_flaw::choose_value::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " val";
}
#endif

void var_flaw::choose_value::apply()
{
}
} // namespace ratio