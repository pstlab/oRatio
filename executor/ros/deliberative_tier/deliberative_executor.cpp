#include "deliberative_executor.h"
#include "deliberative_manager.h"
#include "predicate.h"
#include "atom.h"
#include "deliberative_messages/deliberative_state.h"
#include "deliberative_messages/time.h"
#include "deliberative_messages/timelines.h"
#include "deliberative_services/can_start.h"
#include "deliberative_services/start_task.h"
#include <ros/ros.h>
#include <sstream>

using namespace ratio;

namespace sir
{
    deliberative_executor::deliberative_executor(deliberative_manager &d_mngr, const uint64_t &id, const std::vector<std::string> &domain_files, const std::vector<std::string> &relevant_predicates) : d_mngr(d_mngr), reasoner_id(id), slv(), exec(slv, relevant_predicates), dcl(*this), del(*this)
    {
        // we read the domain files..
        ROS_DEBUG("[%lu] Reading domain..", reasoner_id);
        for (const auto &domain_file : domain_files)
        {
            ROS_DEBUG("[%lu] %s", reasoner_id, domain_file.c_str());
        }

        slv.read(domain_files);

        set_state(Idle);
    }
    deliberative_executor::~deliberative_executor() {}

    void deliberative_executor::started_solving()
    {
        ROS_DEBUG("[%lu] Started reasoning..", reasoner_id);
        set_state(Reasoning);
    }
    void deliberative_executor::solution_found()
    {
        ROS_DEBUG("[%lu] Solution found..", reasoner_id);
        set_state(Executing);

        deliberative_messages::timelines timelines_msg;
        timelines_msg.reasoner_id = reasoner_id;
        const auto tls = slv.extract_timelines();
        const smt::array_val &tls_array = static_cast<const smt::array_val &>(*tls);
        for (size_t i = 0; i < tls_array.size(); ++i)
        {
            std::stringstream ss;
            ss << tls_array.get(i);
            timelines_msg.timelines.push_back(ss.str());
        }

        d_mngr.notify_timelines.publish(timelines_msg);
    }
    void deliberative_executor::inconsistent_problem()
    {
        ROS_DEBUG("[%lu] Inconsistent problem..", reasoner_id);
        set_state(Inconsistent);
    }

    void deliberative_executor::tick(const smt::rational &time)
    {
        ROS_DEBUG("Current time: %s", to_string(time).c_str());
        deliberative_messages::time time_msg;
        time_msg.reasoner_id = reasoner_id;
        time_msg.num = time.numerator();
        time_msg.den = time.denominator();
        d_mngr.notify_time.publish(time_msg);

        arith_expr horizon = slv.get("horizon");
        if (slv.arith_value(horizon) <= exec.get_current_time() && current_tasks.empty())
        {
            ROS_DEBUG("[%lu] Exhausted plan..", reasoner_id);
            state = Finished;
            set_state(Finished);
        }
    }

    void deliberative_executor::starting(const std::unordered_set<atom *> &atms)
    { // tell the executor the atoms which are not yet ready to start..
        std::unordered_set<ratio::atom *> dsy;
        deliberative_services::can_start cs_srv;
        task t;
        for (const auto &atm : atms)
        {
            t = to_task(*atm);
            cs_srv.request.task_name = t.task_name;
            cs_srv.request.par_names = t.par_names;
            cs_srv.request.par_values = t.par_values;
            if (d_mngr.can_start.call(cs_srv) && !cs_srv.response.can_start)
                dsy.insert(atm);
        }

        if (!dsy.empty())
            exec.dont_start_yet(atms);
    }
    void deliberative_executor::start(const std::unordered_set<atom *> &atms)
    { // these atoms are now started..
        deliberative_services::start_task st_srv;
        task t;
        for (const auto &atm : atms)
        {
            ROS_DEBUG("[%lu] Starting task %s..", reasoner_id, atm->get_type().get_name().c_str());
            t = to_task(*atm);
            st_srv.request.reasoner_id = reasoner_id;
            st_srv.request.task_id = t.task_id;
            st_srv.request.task_name = t.task_name;
            st_srv.request.par_names = t.par_names;
            st_srv.request.par_values = t.par_values;
            if (d_mngr.start_task.call(st_srv) && st_srv.response.started)
                current_tasks.emplace(atm->get_sigma(), atm);
        }
    }

    void deliberative_executor::ending(const std::unordered_set<atom *> &atms)
    { // tell the executor the atoms which are not yet ready to finish..
        std::unordered_set<ratio::atom *> dey;
        for (const auto &atm : atms)
            if (current_tasks.count(atm->get_sigma()))
                dey.insert(atm);

        if (!dey.empty())
            exec.dont_end_yet(atms);
    }
    void deliberative_executor::end(const std::unordered_set<atom *> &atms)
    { // these atoms are now ended..
        for (const auto &atm : atms)
        {
            ROS_DEBUG("[%lu] Ended task %s..", reasoner_id, atm->get_type().get_name().c_str());
        }
    }

    void deliberative_executor::finish_task(const smt::var &id, const bool &success)
    {
        ROS_DEBUG("[%lu] Ending task %s..", reasoner_id, current_tasks.at(id)->get_type().get_name().c_str());
        if (!success) // the task failed..
            exec.failure({current_tasks.at(id)});
        current_tasks.erase(id);
    }

    void deliberative_executor::set_state(const executor_state &st)
    {
        state = st;
        deliberative_messages::deliberative_state state_msg;
        state_msg.reasoner_id = reasoner_id;
        state_msg.deliberative_state = st;
        d_mngr.notify_state.publish(state_msg);
    }

    deliberative_executor::task deliberative_executor::to_task(const ratio::atom &atm) const
    {
        uint64_t task_id = atm.get_sigma();
        std::string task_name = atm.get_type().get_name();
        std::vector<std::string> par_names;
        std::vector<std::string> par_values;
        for (const auto &xpr : atm.get_exprs())
            if (xpr.first != START && xpr.first != END && xpr.first != AT && xpr.first != TAU)
            {
                par_names.push_back(xpr.first);
                if (bool_item *bi = dynamic_cast<bool_item *>(&*xpr.second))
                    switch (atm.get_core().bool_value(bi))
                    {
                    case smt::True:
                        par_values.push_back("true");
                        break;
                    case smt::False:
                        par_values.push_back("false");
                        break;
                    default:
                        break;
                    }
                else if (arith_item *ai = dynamic_cast<arith_item *>(&*xpr.second))
                    par_values.push_back(to_string(atm.get_core().arith_value(ai).get_rational()));
                else if (string_item *si = dynamic_cast<string_item *>(&*xpr.second))
                    par_values.push_back(si->get_value());
            }
        return {task_id, task_name, par_names, par_values};
    }
} // namespace sir