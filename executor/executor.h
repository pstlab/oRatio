#pragma once

#include "executor_export.h"
#include "core_listener.h"
#include "solver_listener.h"
#include "inf_rational.h"
#include "lit.h"

namespace ratio
{
  class solver;
  class predicate;
  class atom;
  class executor_listener;

  struct atom_adaptation
  {
    struct exec_bounds
    {
      smt::inf_rational lb, ub;
    };
    smt::lit sigma_xi;
    std::unordered_map<arith_item *, exec_bounds> bounds;
  };

  class executor final : public core_listener, public solver_listener, public smt::theory
  {
    friend class executor_listener;

  public:
    EXECUTOR_EXPORT executor(solver &slv, const smt::rational &units_per_tick = smt::rational::ONE);
    executor(const executor &orig) = delete;

    solver &get_solver() { return slv; }
    const smt::rational &get_current_time() const { return current_time; };

    EXECUTOR_EXPORT atom &get_atom(const smt::var &sigma) const { return *all_atoms.at(sigma); }

    EXECUTOR_EXPORT void tick();

    EXECUTOR_EXPORT void dont_start_yet(const std::unordered_set<atom *> &atoms) { dont_start.insert(atoms.cbegin(), atoms.cend()); }
    EXECUTOR_EXPORT void dont_end_yet(const std::unordered_set<atom *> &atoms) { dont_end.insert(atoms.cbegin(), atoms.cend()); }
    EXECUTOR_EXPORT void failure(const std::unordered_set<atom *> &atoms);

  private:
    bool propagate(const smt::lit &p) noexcept override;
    bool check() noexcept override { return true; }
    void push() noexcept override {}
    void pop() noexcept override {}

    inline bool is_relevant(const predicate &pred) const noexcept { return relevant_predicates.count(&pred); }

    void read(const std::string &) override { reset_relevant_predicates(); }
    void read(const std::vector<std::string> &) override { reset_relevant_predicates(); }
    void solution_found() override;
    void inconsistent_problem() override;

    void flaw_created(const flaw &f) override;

    void build_timelines();
    void freeze(atom &atm, arith_expr &xpr);

    void reset_relevant_predicates();

  private:
    std::unordered_set<const predicate *> relevant_predicates;              // impulses and intervals..
    smt::rational current_time;                                             // the current time in plan units..
    const smt::rational units_per_tick;                                     // the number of plan units for each tick..
    smt::lit xi;                                                            // the execution variable..
    std::unordered_map<const atom *, atom_adaptation> adaptations;          // for each atom, the numeric adaptations done during the executions (i.e., freezes and delays)..
    std::unordered_map<smt::var, atom *> all_atoms;                         // all the interesting atoms indexed by their sigma variable..
    std::unordered_set<const atom *> dont_start;                            // the starting atoms which are not yet ready to start..
    std::unordered_set<const atom *> dont_end;                              // the ending atoms which are not yet ready to end..
    std::map<smt::inf_rational, std::unordered_set<atom *>> s_atms, e_atms; // for each pulse, the atoms starting/ending at that pulse..
    std::set<smt::inf_rational> pulses;                                     // all the pulses of the plan..
    std::vector<executor_listener *> listeners;                             // the executor listeners..
  };

  class execution_exception : public std::exception
  {
    const char *what() const noexcept override { return "the plan cannot be executed.."; }
  };
} // namespace ratio