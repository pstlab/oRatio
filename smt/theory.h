#pragma once

#include "lit.h"
#include <vector>

namespace smt
{
  class sat_core;

  class theory
  {
    friend class sat_core;

  public:
    SMT_EXPORT theory(sat_core &sat);
    theory(const theory &orig) = delete;
    SMT_EXPORT virtual ~theory();

    inline sat_core &get_core() const noexcept { return sat; }

  protected:
    SMT_EXPORT void bind(const var &v) noexcept;
    SMT_EXPORT void swap_conflict(theory &th) noexcept;
    SMT_EXPORT bool backtrack_analyze_and_backjump() noexcept; // backtracks to the proper level before calling analyze_and_backjump..
    SMT_EXPORT void record(const std::vector<lit> &clause) noexcept;

  private:
    /**
     * @brief Analyzes the current conflict and backjumps to the proper decision level.
     */
    void analyze_and_backjump() noexcept;
    /**
     * @brief Asks the theory to perform propagation after the given literal has been assigned. Returns true if the propagation succeeds or false if an inconsistency is found. In case of inconsistency, the confl vector must be filled with the conflicting constraint.
     *
     * @param p the literal that has been assigned.
     * @return true if propagation succeeds or false if an inconsistency is found.
     */
    virtual bool propagate(const lit &p) = 0;

    /**
     * @brief Checks whether the theory is consistent with the given propositional assignments. Returns true if the theory is consistent or false if an inconsistency is found. In case of inconsistency, the confl vector must be filled with the conflicting constraint.
     *
     * @return true if the theory is consistent or false if an inconsistency is found.
     */
    virtual bool check() = 0;

    /**
     * @brief Notifies the theory that some information for subsequent backtracking might need to be stored.
     */
    virtual void push() = 0;

    /**
     * @brief Notifies the theory that a backtracking step is required.
     */
    virtual void pop() = 0;

  protected:
    sat_core &sat;
    std::vector<lit> cnfl;
  };
} // namespace smt