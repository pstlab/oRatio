#pragma once

#include "constr.h"

namespace smt
{

  class sat_core;

  /**
   * This class is used for representing propositional conjunctions.
   */
  class conj : public constr
  {
    friend class sat_core;

  private:
    conj(sat_core &s, const lit &ctr, const std::vector<lit> &lits);
    conj(const conj &orig) = delete;
    ~conj() override;

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