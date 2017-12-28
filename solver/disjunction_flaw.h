#pragma once

#include "flaw.h"
#include "resolver.h"
#include "context.h"

namespace ratio
{

class disjunction;
class conjunction;

class disjunction_flaw : public flaw
{
public:
  disjunction_flaw(solver &slv, resolver *const cause, const context &ctx, const disjunction &disj);
  disjunction_flaw(const disjunction_flaw &orig) = delete;
  virtual ~disjunction_flaw();

  std::string get_label() const override { return "φ" + std::to_string(get_phi()) + " disj"; }

private:
  void compute_resolvers() override;

  class choose_conjunction : public resolver
  {
  public:
    choose_conjunction(solver &slv, disjunction_flaw &disj_flaw, const context &ctx, const conjunction &conj);
    choose_conjunction(const choose_conjunction &that) = delete;
    virtual ~choose_conjunction();

    std::string get_label() const override { return "ρ" + std::to_string(rho) + " conj"; }

  private:
    void apply() override;

  private:
    context ctx;
    const conjunction &conj;
  };

private:
  context ctx;
  const disjunction &disj;
};
}