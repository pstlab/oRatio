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

  class executor : public core_listener, public solver_listener, public smt::theory
  {
    friend class executor_listener;

  public:
    EXECUTOR_EXPORT executor(solver &slv, const std::string &cnfg_str = "{}", const smt::rational &units_per_tick = smt::rational::ONE);
    executor(const executor &orig) = delete;
    EXECUTOR_EXPORT ~executor();

    smt::rational get_current_time() const { return current_time; };

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

    inline bool is_impulse(const atom &atm) const noexcept;
    inline bool is_interval(const atom &atm) const noexcept;
    inline bool is_relevant(const predicate &pred) const noexcept { return relevant_predicates.count(&pred); }

    void read(const std::string &) override { reset_relevant_predicates(); }
    void read(const std::vector<std::string> &) override { reset_relevant_predicates(); }
    void solution_found() override;
    void inconsistent_problem() override;

    void flaw_created(const flaw &f) override;

    void reset_relevant_predicates();
    void build_timelines();
    void freeze(atom &atm, arith_expr &xpr);

  private:
    const std::string cnfg_str;
    const predicate &int_pred;
    const predicate &imp_pred;
    std::unordered_set<const predicate *> relevant_predicates;
    smt::rational current_time;         // the current time in plan units..
    const smt::rational units_per_tick; // the number of plan units for each tick..
    std::unordered_map<const atom *, std::unordered_map<arith_item *, const smt::inf_rational>> lbs, ubs;
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