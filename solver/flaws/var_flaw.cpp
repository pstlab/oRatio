#include "var_flaw.h"
#include "solver.h"
#include "item.h"

namespace ratio
{
    var_flaw::var_flaw(solver &slv, const std::vector<resolver *> &causes, var_item &v_itm) : flaw(slv, causes, true), v_itm(v_itm) {}
    var_flaw::~var_flaw() {}

    std::string var_flaw::get_label() const noexcept { return "{\"type\":\"enum\", \"phi\":\"" + to_string(get_phi()) + "\", \"position\":" + std::to_string(get_position()) + "}"; }

    void var_flaw::compute_resolvers()
    {
        std::unordered_set<const smt::var_value *> vals = get_solver().get_ov_theory().value(v_itm.ev);
        for (const auto &v : vals)
            add_resolver(*new choose_value(get_solver(), smt::rational(1, static_cast<smt::I>(vals.size())), *this, *v));
    }

    var_flaw::choose_value::choose_value(solver &slv, smt::rational cst, var_flaw &enm_flaw, const smt::var_value &val) : resolver(slv, slv.get_ov_theory().allows(enm_flaw.v_itm.ev, val), cst, enm_flaw), v(enm_flaw.v_itm.ev), val(val) {}
    var_flaw::choose_value::~choose_value() {}

    std::string var_flaw::choose_value::get_label() const noexcept { return "{\"rho\":\"" + to_string(get_rho()) + "\"}"; }

    void var_flaw::choose_value::apply() {}
} // namespace ratio