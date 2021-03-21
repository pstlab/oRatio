#pragma once

#include "flaw.h"
#include "resolver.h"

namespace ratio
{
  class disj_flaw : public flaw
  {
  public:
    disj_flaw(solver &slv, const std::vector<resolver *> &causes, const std::vector<smt::lit> &lits);
    disj_flaw(const disj_flaw &orig) = delete;
    virtual ~disj_flaw();

    std::string get_label() const noexcept override;

  private:
    void compute_resolvers() override;

    class choose_lit : public resolver
    {
    public:
      choose_lit(solver &slv, smt::rational cst, disj_flaw &disj_flaw, const smt::lit &p);
      choose_lit(const choose_lit &that) = delete;
      virtual ~choose_lit();

      std::string get_label() const noexcept override;

    private:
      void apply() override;
    };

  private:
    std::vector<smt::lit> lits; // the disjunction..
  };
} // namespace ratio