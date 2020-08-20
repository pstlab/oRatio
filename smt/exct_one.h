#pragma once

#include "constr.h"

namespace smt
{

  class sat_core;

  /**
   * This class is used for representing propositional exact-one constraints.
   */
  class exct_one : public constr
  {
    friend class sat_core;

  private:
    exct_one(sat_core &s, const lit &ctr, const std::vector<lit> &lits);
    exct_one(const exct_one &orig) = delete;
    ~exct_one() override;

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