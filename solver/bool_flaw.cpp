#include "bool_flaw.h"
#include "solver.h"
#include "item.h"

namespace ratio
{

    static inline const std::vector<resolver *> cause_to_vector(resolver *const cause)
    {
        if (cause)
            return {cause};
        else
            return {};
    }

    bool_flaw::bool_flaw(graph &gr, resolver *const cause, bool_item &b_itm) : flaw(gr, cause_to_vector(cause), true), b_itm(b_itm) {}
    bool_flaw::~bool_flaw() {}

    std::string bool_flaw::get_label() const noexcept { return "{\"type\":\"enum\", \"phi\":" + to_string(get_phi()) + ", \"position\":" + std::to_string(get_position()) + "}"; }

    void bool_flaw::compute_resolvers()
    {
        add_resolver(*new choose_value(get_graph(), smt::rational(1, 2), *this, b_itm.l));
        add_resolver(*new choose_value(get_graph(), smt::rational(1, 2), *this, !b_itm.l));
    }

    bool_flaw::choose_value::choose_value(graph &gr, smt::rational cst, bool_flaw &bl_flaw, smt::lit &val) : resolver(gr, val, cst, bl_flaw) {}
    bool_flaw::choose_value::~choose_value() {}

    std::string bool_flaw::choose_value::get_label() const noexcept { return "{\"rho\":" + to_string(get_rho()) + "}"; }

    void bool_flaw::choose_value::apply() {}
} // namespace ratio