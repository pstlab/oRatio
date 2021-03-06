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
  disjunction_flaw(graph &gr, resolver *const cause, const context &ctx, const disjunction &disj);
  disjunction_flaw(const disjunction_flaw &orig) = delete;
  virtual ~disjunction_flaw();

#ifdef BUILD_GUI
  std::string get_label() const override;
#endif

private:
  void compute_resolvers() override;

  class choose_conjunction : public resolver
  {
  public:
    choose_conjunction(graph &gr, disjunction_flaw &disj_flaw, const context &ctx, const conjunction &conj);
    choose_conjunction(const choose_conjunction &that) = delete;
    virtual ~choose_conjunction();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

  private:
    void apply() override;

  private:
    context ctx;             // the context for executing the conjunction..
    const conjunction &conj; // the conjunction to execute..
  };

private:
  context ctx;             // the context for executing the disjunction..
  const disjunction &disj; // the disjunction to execute..
};
} // namespace ratio