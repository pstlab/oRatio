#pragma once

#include "constr.h"

namespace smt
{
  class sat_core;
  class theory;

  /**
   * This class is used for representing propositional clauses.
   */
  class clause : public constr
  {
    friend class sat_core;
    friend class theory;

  private:
    clause(sat_core &s, const std::vector<lit> &lits);
    clause(const clause &orig) = delete;
    ~clause() override;

  public:
    inline const std::vector<lit> get_lits() const noexcept { return lits; }

  private:
    bool propagate(const lit &p) noexcept override;
    bool simplify() noexcept override;
    void remove() noexcept override;
    void get_reason(const lit &p, std::vector<lit> &out_reason) const noexcept override;

    json to_json() const noexcept override;

  private:
    std::vector<lit> lits;
  };
} // namespace smt