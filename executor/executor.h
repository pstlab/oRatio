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
    EXECUTOR_EXPORT executor(solver &slv, const smt::rational &units_per_tick = smt::rational::ONE);
    executor(const executor &orig) = delete;
    EXECUTOR_EXPORT ~executor();

    smt::rational get_current_time() const { return current_time; };

    EXECUTOR_EXPORT atom &get_atom(const smt::var &sigma) const { return *all_atoms.at(sigma); }

    EXECUTOR_EXPORT void tick();

    EXECUTOR_EXPORT void freeze(const atom &atm, const expr &xpr);
    EXECUTOR_EXPORT void done(const std::set<atom *> &atoms);
    EXECUTOR_EXPORT void failure(const std::set<atom *> &atoms);

  private:
    bool propagate(const smt::lit &p) noexcept override { return true; }
    bool check() noexcept override { return true; }
    void push() noexcept override {}
    void pop() noexcept override { should_set = true; }

    inline bool is_impulse(const atom &atm) const noexcept;
    inline bool is_interval(const atom &atm) const noexcept;

    void solution_found() override;
    void inconsistent_problem() override;

    void flaw_created(const flaw &f) override;

    void build_timelines();
    void set_values();

  private:
    const predicate &int_pred;
    const predicate &imp_pred;
    bool should_set = false;
    smt::rational current_time;         // the current time in plan units..
    const smt::rational units_per_tick; // the number of plan units for each tick..
    std::unordered_map<const atom *, std::unordered_map<arith_item *, smt::inf_rational>> lbs;
    std::unordered_map<const atom *, std::unordered_map<arith_item *, smt::inf_rational>> ubs;
    std::unordered_map<const var_item *, const smt::var_value *> frozen_vals;
    std::unordered_map<smt::var, atom *> all_atoms;               // all the interesting atoms indexed by their sigma variable..
    std::set<atom *> executing;                                   // the currently executing atoms..
    std::map<smt::inf_rational, std::set<atom *>> s_atms, e_atms; // for each pulse, the atoms starting/ending at that pulse..
    std::set<smt::inf_rational> pulses;                           // all the pulses of the plan..
    std::vector<executor_listener *> listeners;                   // the executor listeners..
  };

  class execution_exception : public std::exception
  {
    const char *what() const noexcept override { return "the plan cannot be executed.."; }
  };
} // namespace ratio