#include "bool_flaw.h"
#include "solver.h"
#include "item.h"

namespace ratio
{
    bool_flaw::bool_flaw(solver &slv, const std::vector<resolver *> &causes, bool_item &b_itm) : flaw(slv, causes, true), b_itm(b_itm) {}
    bool_flaw::~bool_flaw() {}

    std::string bool_flaw::get_label() const noexcept { return "{\"type\":\"bool\", \"phi\":\"" + to_string(get_phi()) + "\", \"position\":" + std::to_string(get_position()) + "}"; }

    void bool_flaw::compute_resolvers()
    {
        add_resolver(*new choose_value(get_solver(), smt::rational(1, 2), *this, b_itm.l));
        add_resolver(*new choose_value(get_solver(), smt::rational(1, 2), *this, !b_itm.l));
    }

    bool_flaw::choose_value::choose_value(solver &slv, smt::rational cst, bool_flaw &bl_flaw, const smt::lit &val) : resolver(slv, val, cst, bl_flaw) {}
    bool_flaw::choose_value::~choose_value() {}

    std::string bool_flaw::choose_value::get_label() const noexcept { return "{\"rho\":\"" + to_string(get_rho()) + "\"}"; }

    void bool_flaw::choose_value::apply() {}
} // namespace ratio