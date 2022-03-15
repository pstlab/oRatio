#pragma once

#include "executor_listener.h"

namespace ratio
{
  class deliberative_manager;

  class deliberative_executor
  {
    friend class deliberative_manager;

  public:
    deliberative_executor(deliberative_manager &d_mngr, const uint64_t &id, const std::vector<std::string> &domain_files, std::vector<std::string> notify_start_ids = {});
    ~deliberative_executor();

    uint64_t get_reasoner_id() { return reasoner_id; }
    ratio::solver &get_solver() { return slv; }
    ratio::executor &get_executor() { return exec; }

    void finish_task(const smt::var &id, const bool &success = true);

  private:
    void set_state(const unsigned int &state);
    predicate &get_predicate(const std::string &pred) const;

    struct task
    {
      uint64_t task_id;
      std::string task_name;
      std::vector<std::string> par_names;
      std::vector<std::string> par_values;
    };

    static task to_task(const ratio::atom &atm);

    class deliberative_core_listener : public ratio::core_listener
    {
    public:
      deliberative_core_listener(deliberative_executor &de) : exec(de), core_listener(de.get_solver()) {}
      ~deliberative_core_listener() {}

    private:
      void read(const std::string &) override { reset_relevant_predicates(); }
      void read(const std::vector<std::string> &) override { reset_relevant_predicates(); }

      void started_solving() override;
      void solution_found() override;
      void inconsistent_problem() override;

      void reset_relevant_predicates();

    private:
      deliberative_executor &exec;
    };

    class deliberative_executor_listener : public ratio::executor_listener
    {
    public:
      deliberative_executor_listener(deliberative_executor &de) : exec(de), executor_listener(de.get_executor()) {}
      ~deliberative_executor_listener() {}

    private:
      void tick(const smt::rational &time) override;

      void starting(const std::unordered_set<ratio::atom *> &atms) override;
      void start(const std::unordered_set<ratio::atom *> &atms) override;

      void ending(const std::unordered_set<ratio::atom *> &atms) override;
      void end(const std::unordered_set<ratio::atom *> &atms) override;

    private:
      deliberative_executor &exec;
    };

    class deliberative_solver_listener : public ratio::solver_listener
    {
    public:
      deliberative_solver_listener(deliberative_executor &de) : exec(de), solver_listener(de.get_solver()) {}
      ~deliberative_solver_listener() {}

    private:
      void flaw_created(const flaw &f) override;
      void flaw_state_changed(const flaw &f) override;
      void flaw_cost_changed(const flaw &f) override;
      void flaw_position_changed(const flaw &f) override;
      void current_flaw(const flaw &f) override;

      void resolver_created(const resolver &r) override;
      void resolver_state_changed(const resolver &r) override;
      void current_resolver(const resolver &r) override;

      void causal_link_added(const flaw &f, const resolver &r) override;

    private:
      deliberative_executor &exec;
    };

  private:
    deliberative_manager &d_mngr;
    uint64_t reasoner_id;
    ratio::solver slv;
    ratio::executor exec;
    deliberative_core_listener dcl;
    deliberative_solver_listener dsl;
    deliberative_executor_listener del;
    unsigned int state = -1;
    std::vector<std::string> notify_start_ids;
    std::unordered_set<const ratio::predicate *> notify_start;
    std::unordered_map<smt::var, ratio::atom *> current_tasks;
    std::unordered_set<const flaw *> flaws;
    std::unordered_set<const resolver *> resolvers;
    const flaw *current_flaw = nullptr;
    const resolver *current_resolver = nullptr;
    smt::rational current_time;
    std::unordered_set<ratio::atom *> executing;
  };
} // namespace ratio
