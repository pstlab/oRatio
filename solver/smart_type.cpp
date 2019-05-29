#include "smart_type.h"
#include "solver.h"
#include "atom.h"
#include "field.h"
#include "atom_flaw.h"
#include "combinations.h"
#include <cassert>

namespace ratio
{

smart_type::smart_type(solver &slv, scope &scp, const std::string &name) : type(slv, scp, name, false), slv(slv) {}
smart_type::~smart_type() {}

std::vector<flaw *> smart_type::get_flaws()
{
    assert(slv.get_sat_core().root_level());
    return std::move(flaws);
}

void smart_type::new_fact(atom_flaw &af) {}
void smart_type::new_goal(atom_flaw &af) {}

smart_type::smart_flaw::smart_flaw(smart_type &st, const std::set<atom *> &atms) : flaw(st.get_solver().get_graph(), smart_type::get_resolvers(st.get_solver(), atms)), st(st), atms(atms) {}
smart_type::smart_flaw::~smart_flaw() {}

#ifdef BUILD_GUI
std::string smart_type::smart_flaw::get_label() const
{
    return "φ" + std::to_string(get_phi()) + " sv-flaw";
}
#endif

void smart_type::smart_flaw::compute_resolvers()
{
    std::vector<std::vector<atom *>> cs = combinations(std::vector<atom *>(atms.begin(), atms.end()), 2);
    for (const auto &as : cs)
    {
        auto chs = st.choices.find(std::set<atom *>({as[0], as[1]}));
        for (const auto &ch : chs->second)
#ifdef BUILD_GUI
            add_resolver(*new smart_resolver(*this, ch.first.get_var(), ch.second));
#else
            add_resolver(*new smart_resolver(*this, ch.get_var()));
#endif
    }
}

#ifdef BUILD_GUI
smart_type::smart_resolver::smart_resolver(smart_flaw &flw, const smt::var &r, const std::string &lbl) : resolver(flw.get_graph(), r, 0, flw), lbl(lbl)
{
}
#else
smart_type::smart_resolver::smart_resolver(smart_flaw &flw, const smt::var &r) : resolver(flw.get_graph(), r, 0, flw)
{
}
#endif
smart_type::smart_resolver::~smart_resolver()
{
}

#ifdef BUILD_GUI
std::string smart_type::smart_resolver::get_label() const
{
    return lbl;
}
#endif

void smart_type::smart_resolver::apply()
{
}

void smart_type::set_ni(const smt::var &v) { get_solver().set_ni(v); }
void smart_type::restore_ni() { get_solver().restore_ni(); }

std::vector<resolver *> smart_type::get_resolvers(solver &slv, const std::set<atom *> &atms)
{
    std::unordered_set<resolver *> ress;
    for (const auto &atm : atms)
        for (const auto &r : slv.get_reason(*atm).get_resolvers())
            if (resolver *af = dynamic_cast<atom_flaw::activate_fact *>(r))
                ress.insert(af);
            else if (resolver *ag = dynamic_cast<atom_flaw::activate_goal *>(r))
                ress.insert(ag);

    return std::vector<resolver *>(ress.begin(), ress.end());
}

atom_listener::atom_listener(atom &atm) : smt::sat_value_listener(atm.get_core().get_sat_core()), smt::lra_value_listener(atm.get_core().get_lra_theory()), smt::ov_value_listener(atm.get_core().get_ov_theory()), atm(atm)
{
    std::queue<const type *> q;
    q.push(&atm.get_type());
    while (!q.empty())
    {
        for (const auto &f : q.front()->get_fields())
            if (!f.second->is_synthetic())
            {
                item *i = &*atm.get(f.first);
                if (bool_item *be = dynamic_cast<bool_item *>(i))
                    listen_sat(be->l.get_var());
                else if (arith_item *ae = dynamic_cast<arith_item *>(i))
                    for (const auto &term : ae->l.vars)
                        listen_lra(term.first);
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
