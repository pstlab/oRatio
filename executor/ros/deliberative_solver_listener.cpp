#include "deliberative_solver_listener.h"
#include "deliberative_manager.h"
#include "deliberative_executor.h"
#include "deliberative_tier/flaw_created.h"
#include "deliberative_tier/flaw_state_changed.h"
#include "deliberative_tier/flaw_cost_changed.h"
#include "deliberative_tier/flaw_position_changed.h"
#include "deliberative_tier/current_flaw.h"
#include "deliberative_tier/resolver_created.h"
#include "deliberative_tier/resolver_state_changed.h"
#include "deliberative_tier/current_resolver.h"
#include "deliberative_tier/causal_link_added.h"

using namespace ratio;

namespace ratio
{
    deliberative_solver_listener::deliberative_solver_listener(deliberative_manager &d_mngr, deliberative_executor &d_exec) : solver_listener(d_exec.get_solver()),
                                                                                                                              d_mngr(d_mngr),
                                                                                                                              d_exec(d_exec),
                                                                                                                              flaw_created_pub(d_mngr.get_handle().advertise<deliberative_tier::flaw_created>("flaw_created", 10, true)),
                                                                                                                              flaw_state_changed_pub(d_mngr.get_handle().advertise<deliberative_tier::flaw_state_changed>("flaw_state_changed", 10, true)),
                                                                                                                              flaw_cost_changed_pub(d_mngr.get_handle().advertise<deliberative_tier::flaw_cost_changed>("flaw_cost_changed", 10, true)),
                                                                                                                              flaw_position_changed_pub(d_mngr.get_handle().advertise<deliberative_tier::flaw_position_changed>("flaw_position_changed", 10, true)),
                                                                                                                              current_flaw_pub(d_mngr.get_handle().advertise<deliberative_tier::current_flaw>("current_flaw", 10, true)),
                                                                                                                              resolver_created_pub(d_mngr.get_handle().advertise<deliberative_tier::resolver_created>("resolver_created", 10, true)),
                                                                                                                              resolver_state_changed_pub(d_mngr.get_handle().advertise<deliberative_tier::resolver_state_changed>("resolver_state_changed", 10, true)),
                                                                                                                              current_resolver_pub(d_mngr.get_handle().advertise<deliberative_tier::current_resolver>("current_resolver", 10, true)),
                                                                                                                              causal_link_added_pub(d_mngr.get_handle().advertise<deliberative_tier::causal_link_added>("causal_link_added", 10, true))
    {
    }
    deliberative_solver_listener::~deliberative_solver_listener() {}

    void deliberative_solver_listener::flaw_created(const flaw &f)
    {
        deliberative_tier::flaw_created fc_msg;
        fc_msg.reasoner_id = d_exec.get_reasoner_id();
        fc_msg.flaw_id = reinterpret_cast<std::uintptr_t>(&f);
        for (const auto &r : f.get_causes())
            fc_msg.causes.push_back(reinterpret_cast<std::uintptr_t>(r));
        fc_msg.label = f.get_label();
        fc_msg.state = slv.get_sat_core().value(f.get_phi());
        const auto [lb, ub] = slv.get_idl_theory().bounds(f.get_position());
        fc_msg.lb = lb, fc_msg.ub = ub;
        flaw_created_pub.publish(fc_msg);
    }
    void deliberative_solver_listener::flaw_state_changed(const flaw &f)
    {
        deliberative_tier::flaw_state_changed fsc_msg;
        fsc_msg.reasoner_id = d_exec.get_reasoner_id();
        fsc_msg.flaw_id = reinterpret_cast<std::uintptr_t>(&f);
        fsc_msg.state = slv.get_sat_core().value(f.get_phi());
        flaw_state_changed_pub.publish(fsc_msg);
    }
    void deliberative_solver_listener::flaw_cost_changed(const flaw &f)
    {
        deliberative_tier::flaw_cost_changed fcc_msg;
        fcc_msg.reasoner_id = d_exec.get_reasoner_id();
        fcc_msg.flaw_id = reinterpret_cast<std::uintptr_t>(&f);
        const auto est_cost = f.get_estimated_cost();
        fcc_msg.cost.num = est_cost.numerator(), fcc_msg.cost.den = est_cost.denominator();
        flaw_cost_changed_pub.publish(fcc_msg);
    }
    void deliberative_solver_listener::flaw_position_changed(const flaw &f)
    {
        deliberative_tier::flaw_position_changed fpc_msg;
        fpc_msg.reasoner_id = d_exec.get_reasoner_id();
        fpc_msg.flaw_id = reinterpret_cast<std::uintptr_t>(&f);
        const auto [lb, ub] = slv.get_idl_theory().bounds(f.get_position());
        fpc_msg.lb = lb, fpc_msg.ub = ub;
        flaw_position_changed_pub.publish(fpc_msg);
    }
    void deliberative_solver_listener::current_flaw(const flaw &f)
    {
        deliberative_tier::current_flaw cf_msg;
        cf_msg.reasoner_id = d_exec.get_reasoner_id();
        cf_msg.flaw_id = reinterpret_cast<std::uintptr_t>(&f);
        current_flaw_pub.publish(cf_msg);
    }

    void deliberative_solver_listener::resolver_created(const resolver &r)
    {
        deliberative_tier::resolver_created rc_msg;
        rc_msg.reasoner_id = d_exec.get_reasoner_id();
        rc_msg.resolver_id = reinterpret_cast<std::uintptr_t>(&r);
        rc_msg.effect = reinterpret_cast<std::uintptr_t>(&r.get_effect());
        rc_msg.label = r.get_label();
        rc_msg.state = slv.get_sat_core().value(r.get_rho());
        const auto est_cost = r.get_estimated_cost();
        rc_msg.cost.num = est_cost.numerator(), rc_msg.cost.den = est_cost.denominator();
        resolver_created_pub.publish(rc_msg);
    }
    void deliberative_solver_listener::resolver_state_changed(const resolver &r)
    {
        deliberative_tier::resolver_state_changed rsc_msg;
        rsc_msg.reasoner_id = d_exec.get_reasoner_id();
        rsc_msg.resolver_id = reinterpret_cast<std::uintptr_t>(&r);
        rsc_msg.state = slv.get_sat_core().value(r.get_rho());
        resolver_state_changed_pub.publish(rsc_msg);
    }
    void deliberative_solver_listener::current_resolver(const resolver &r)
    {
        deliberative_tier::current_resolver cr_msg;
        cr_msg.reasoner_id = d_exec.get_reasoner_id();
        cr_msg.resolver_id = reinterpret_cast<std::uintptr_t>(&r);
        current_resolver_pub.publish(cr_msg);
    }

    void deliberative_solver_listener::causal_link_added(const flaw &f, const resolver &r)
    {
        deliberative_tier::causal_link_added cl_msg;
        cl_msg.reasoner_id = d_exec.get_reasoner_id();
        cl_msg.flaw_id = reinterpret_cast<std::uintptr_t>(&f);
        cl_msg.resolver_id = reinterpret_cast<std::uintptr_t>(&r);
        causal_link_added_pub.publish(cl_msg);
    }
} // namespace ratio
