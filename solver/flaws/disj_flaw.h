#pragma once

#include "flaw.h"
#include "resolver.h"

namespace ratio
{
  class disj_flaw final : public flaw
  {
  public:
    disj_flaw(solver &slv, std::vector<resolver *> causes, std::vector<smt::lit> lits);
    disj_flaw(const disj_flaw &orig) = delete;

    std::string get_data() const noexcept override;

  private:
    void compute_resolvers() override;

    class choose_lit final : public resolver
    {
    public:
      choose_lit(solver &slv, smt::rational cst, disj_flaw &disj_flaw, const smt::lit &p);
      choose_lit(const choose_lit &that) = delete;

      std::string get_data() const noexcept override;

    private:
      void apply() override;
    };

  private:
    std::vector<smt::lit> lits; // the disjunction..
  };
} // namespace ratio