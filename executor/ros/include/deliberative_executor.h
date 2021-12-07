#pragma once

#include "executor_listener.h"

namespace ratio
{
  class deliberative_manager;

  enum executor_state
  {
    Idle,
    Reasoning,
    Executing,
    Finished,
    Inconsistent
  };

  class deliberative_executor
  {
  public:
    deliberative_executor(deliberative_manager &d_mngr, const uint64_t &id, const std::vector<std::string> &domain_files, const std::vector<std::string> &relevant_predicates = {});
    ~deliberative_executor();

    uint64_t get_reasoner_id() { return reasoner_id; }
    ratio::solver &get_solver() { return slv; }
    ratio::executor &get_executor() { return exec; }

    void finish_task(const smt::var &id, const bool &success = true);

  private:
    void started_solving();
    void solution_found();
    void inconsistent_problem();

    void tick(const smt::rational &time);

    void starting(const std::unordered_set<ratio::atom *> &);
    void start(const std::unordered_set<ratio::atom *> &);

    void ending(const std::unordered_set<ratio::atom *> &);
    void end(const std::unordered_set<ratio::atom *> &);

    void set_state(const executor_state &state);

    struct task
    {
      uint64_t task_id;
      std::string task_name;
      std::vector<std::string> par_names;
      std::vector<std::string> par_values;
    };

    task to_task(const ratio::atom &atm) const;

    class deliberative_core_listener : public ratio::core_listener
    {
    public:
      deliberative_core_listener(deliberative_executor &de) : exec(de), core_listener(de.get_solver()) {}
      ~deliberative_core_listener() {}

    private:
      void started_solving() override { exec.started_solving(); }
      void solution_found() override { exec.solution_found(); }
      void inconsistent_problem() override { exec.inconsistent_problem(); }

    private:
      deliberative_executor &exec;
    };

    class deliberative_executor_listener : public ratio::executor_listener
    {
    public:
      deliberative_executor_listener(deliberative_executor &de) : exec(de), executor_listener(de.get_executor()) {}
      ~deliberative_executor_listener() {}

    private:
      void tick(const smt::rational &time) override { exec.tick(time); }

      void starting(const std::unordered_set<ratio::atom *> &atms) override { exec.starting(atms); }
      void start(const std::unordered_set<ratio::atom *> &atms) override { exec.start(atms); }

      void ending(const std::unordered_set<ratio::atom *> &atms) override { exec.ending(atms); }
      void end(const std::unordered_set<ratio::atom *> &atms) override { exec.end(atms); }

    private:
      deliberative_executor &exec;
    };

  private:
    deliberative_manager &d_mngr;
    uint64_t reasoner_id;
    ratio::solver slv;
    ratio::executor exec;
    deliberative_core_listener dcl;
    deliberative_executor_listener del;
    executor_state state = Idle;
    std::unordered_map<smt::var, ratio::atom *> current_tasks;
  };
} // namespace ratio
