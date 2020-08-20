#pragma once

#include "constr.h"

namespace smt
{

  class sat_core;

  /**
   * This class is used for representing propositional disjunctions.
   */
  class disj : public constr
  {
    friend class sat_core;

  private:
    disj(sat_core &s, const lit &ctr, const std::vector<lit> &lits);
    disj(const disj &orig) = delete;
    ~disj() override;

  public:
    const lit get_ctr() const { return ctr; }
    const std::vector<lit> get_lits() const { return lits; }

  private:
    const bool propagate(const lit &p) override;
    const bool simplify() override;
    void remove() override;
    void get_reason(const lit &p, std::vector<lit> &out_reason) const override;

  private:
    lit ctr;
    std::vector<lit> lits;
  };
} // namespace smt