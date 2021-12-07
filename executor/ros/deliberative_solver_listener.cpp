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
        fc_msg.lb = lb;
        fc_msg.ub = ub;
        flaw_created_pub.publish(fc_msg);
    }
    void deliberative_solver_listener::flaw_state_changed(const flaw &f)
    {
    }
    void deliberative_solver_listener::flaw_cost_changed(const flaw &f)
    {
    }
    void deliberative_solver_listener::flaw_position_changed(const flaw &f)
    {
    }
    void deliberative_solver_listener::current_flaw(const flaw &f)
    {
    }

    void deliberative_solver_listener::resolver_created(const resolver &r)
    {
    }
    void deliberative_solver_listener::resolver_state_changed(const resolver &r)
    {
    }
    void deliberative_solver_listener::current_resolver(const resolver &r)
    {
    }

    void deliberative_solver_listener::causal_link_added(const flaw &f, const resolver &r)
    {
    }
} // namespace ratio
