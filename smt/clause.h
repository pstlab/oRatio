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
    clause(sat_core &s, std::vector<lit> lits);
    clause(const clause &orig) = delete;
    ~clause() override;

    static clause *new_clause(sat_core &s, std::vector<lit> lits);

  public:
    inline const std::vector<lit> get_lits() const noexcept { return lits; }

  private:
    constr *copy(sat_core &s) noexcept override { return new_clause(s, lits); }
    bool propagate(const lit &p) noexcept override;
    bool simplify() noexcept override;
    void remove() noexcept override;
    void get_reason(const lit &p, std::vector<lit> &out_reason) const noexcept override;

    json to_json() const noexcept override;

  private:
    std::vector<lit> lits;
  };
} // namespace smt