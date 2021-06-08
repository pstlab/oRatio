#include "smart_type.h"
#include "solver.h"
#include "atom.h"
#include "field.h"
#include "atom_flaw.h"
#include "core_parser.h"
#include <cassert>

namespace ratio
{
    smart_type::smart_type(solver &slv, scope &scp, const std::string &name) : type(slv, scp, name, false), slv(slv) {}
    smart_type::~smart_type() {}

    void smart_type::new_atom(atom_flaw &) {}

    void smart_type::set_ni(const smt::lit &v) noexcept { get_solver().set_ni(v); }
    void smart_type::restore_ni() noexcept { get_solver().restore_ni(); }

    void smart_type::store_flaw(flaw &f) noexcept { slv.pending_flaws.push_back(&f); }

    std::vector<resolver *> smart_type::get_resolvers(solver &slv, const std::set<atom *> &atms) noexcept
    {
        std::unordered_set<resolver *> ress;
        for (const auto &atm : atms)
            for (const auto &r : slv.get_reason(*atm).get_resolvers())
                if (resolver *af = dynamic_cast<atom_flaw::activate_fact *>(r))
                    ress.insert(af);
                else if (resolver *ag = dynamic_cast<atom_flaw::activate_goal *>(r))
                    ress.insert(ag);

        return std::vector<resolver *>(ress.cbegin(), ress.cend());
    }

    atom_listener::atom_listener(atom &atm) : smt::sat_value_listener(atm.get_core().get_sat_core()), smt::lra_value_listener(atm.get_core().get_lra_theory()), smt::rdl_value_listener(atm.get_core().get_rdl_theory()), smt::ov_value_listener(atm.get_core().get_ov_theory()), atm(atm)
    {
        std::queue<const type *> q;
        q.push(&atm.get_type());
        while (!q.empty())
        {
            for (const auto &[f_name, f] : q.front()->get_fields())
                if (!f->is_synthetic())
                {
                    item *i = &*atm.get(f_name);
                    if (bool_item *be = dynamic_cast<bool_item *>(i))
                        listen_sat(variable(be->l));
                    else if (arith_item *ae = dynamic_cast<arith_item *>(i))
                    {
                        if (ae->get_type().get_name().compare(TP_KEYWORD) == 0)
                            for (const auto &[v, c] : ae->l.vars)
                                listen_rdl(v);
                        else
                            for (const auto &[v, c] : ae->l.vars)
                                listen_lra(v);
                    }
                    else if (var_item *ee = dynamic_cast<var_item *>(i))
                        listen_set(ee->ev);
                }

            for (const auto &st : q.front()->get_supertypes())
                q.push(st);
            q.pop();
        }
    }
    atom_listener::~atom_listener() {}
} // namespace ratio