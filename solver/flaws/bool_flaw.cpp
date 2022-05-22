#include "bool_flaw.h"
#include "solver.h"
#include "item.h"

namespace ratio
{
    bool_flaw::bool_flaw(solver &slv, std::vector<resolver *> causes, bool_item &b_itm) : flaw(slv, std::move(causes), true), b_itm(b_itm) {}

    std::string bool_flaw::get_data() const noexcept { return "{\"type\":\"bool\", \"phi\":\"" + to_string(get_phi()) + "\", \"position\":" + std::to_string(get_position()) + "}"; }

    void bool_flaw::compute_resolvers()
    {
        add_resolver(*new choose_value(smt::rational(1, 2), *this, b_itm.l));
        add_resolver(*new choose_value(smt::rational(1, 2), *this, !b_itm.l));
    }

    bool_flaw::choose_value::choose_value(smt::rational cst, bool_flaw &bl_flaw, const smt::lit &val) : resolver(val, cst, bl_flaw) {}

    std::string bool_flaw::choose_value::get_data() const noexcept { return "{\"rho\":\"" + to_string(get_rho()) + "\"}"; }

    void bool_flaw::choose_value::apply() {}
} // namespace ratio