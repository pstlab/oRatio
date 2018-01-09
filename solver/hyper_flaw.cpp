#include "hyper_flaw.h"
#include "atom_flaw.h"
#include "solver.h"
#include "cartesian_product.h"
#include "combinations.h"
#include <cassert>

namespace ratio
{

inline const std::vector<resolver *> get_cause(resolver *const cause)
{
    if (cause)
        return {cause};
    else
        return {};
}

hyper_flaw::hyper_flaw(solver &slv, resolver *const cause, const std::vector<flaw *> &fs) : flaw(slv, get_cause(cause)), flaws(fs)
{
    std::set<flaw *> c_fs(fs.begin(), fs.end());
    assert(slv.hyper_flaws.find(c_fs) == slv.hyper_flaws.end());
    slv.hyper_flaws.insert({c_fs, this});
}
hyper_flaw::~hyper_flaw() {}

void hyper_flaw::compute_resolvers()
{
    std::vector<std::vector<resolver *>> rs;
    for (const auto &f : flaws)
        rs.push_back(f->get_resolvers());
    for (const auto &rp : cartesian_product(rs))
    {
        // the resolver's cost is given by the maximum of the enclosing resolvers' costs..
        smt::rational cst(smt::rational::NEGATIVE_INFINITY);
        std::vector<smt::lit> cnj;
        for (const auto &r : rp)
        {
            if (cst < r->get_intrinsic_cost())
                cst = r->get_intrinsic_cost();
            cnj.push_back(r->get_rho());
        }
        smt::var cnj_var = slv.sat_cr.new_conj(cnj);
        if (slv.sat_cr.check({cnj_var}))
            add_resolver(*new hyper_resolver(slv, *this, cnj_var, cst, rp));
    }
}

hyper_flaw::hyper_resolver::hyper_resolver(solver &slv, hyper_flaw &s_flaw, const smt::var &app_r, const smt::rational &cst, const std::vector<resolver *> &rs) : resolver(slv, app_r, cst, s_flaw), resolvers(rs) {}
hyper_flaw::hyper_resolver::~hyper_resolver() {}
void hyper_flaw::hyper_resolver::apply()
{
    // all the resolver's preconditions..
    std::vector<flaw *> precs;
    for (const auto &r : resolvers)
        for (const auto &pre : r->get_preconditions())
            precs.push_back(pre);

    if (precs.size() > slv.accuracy) // we create sets having the size of the accuracy..
    {
        std::vector<std::vector<flaw *>> fss = combinations(std::vector<flaw *>(precs.begin(), precs.end()), slv.accuracy);
        for (const auto &fs : fss) // we create a new super flaw for each of the possible combinations..
        {
            std::set<flaw *> c_fs(fs.begin(), fs.end());
            const auto &at_sf = slv.hyper_flaws.find(c_fs);
            if (at_sf != slv.hyper_flaws.end())
            {
                slv.new_causal_link(*at_sf->second, *this);
                slv.set_estimated_cost(*this, at_sf->second->get_estimated_cost());
            }
            else
                slv.new_flaw(*new hyper_flaw(slv, this, fs));
        }
    }
    else if (!precs.empty()) // we create a new super flaw including all the preconditions of this resolver..
    {
        std::set<flaw *> c_fs(precs.begin(), precs.end());
        const auto &at_sf = slv.hyper_flaws.find(c_fs);
        if (at_sf != slv.hyper_flaws.end())
        {
            slv.new_causal_link(*at_sf->second, *this);
            slv.set_estimated_cost(*this, at_sf->second->get_estimated_cost());
        }
        else
            slv.new_flaw(*new hyper_flaw(slv, this, precs));
    }
}
}