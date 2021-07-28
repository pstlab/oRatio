#pragma once

#include "flaw.h"
#include "resolver.h"
#include "context.h"

namespace ratio
{
  class disjunction;
  class conjunction;

  class disjunction_flaw final : public flaw
  {
  public:
    disjunction_flaw(solver &slv, std::vector<resolver *> causes, const context &ctx, const std::vector<const conjunction *> &conjs);
    disjunction_flaw(const disjunction_flaw &orig) = delete;

    std::string get_label() const noexcept override;

  private:
    void compute_resolvers() override;

    class choose_conjunction final : public resolver
    {
    public:
      choose_conjunction(solver &slv, disjunction_flaw &disj_flaw, const context &ctx, const conjunction &conj);
      choose_conjunction(const choose_conjunction &that) = delete;

      std::string get_label() const noexcept override;

    private:
      void apply() override;

    private:
      context ctx;             // the context for executing the conjunction..
      const conjunction &conj; // the conjunction to execute..
    };

  private:
    context ctx;                                  // the context for executing the disjunction..
    const std::vector<const conjunction *> conjs; // the disjunction to execute..
  };
} // namespace ratio