#pragma once

#include "graph.h"

namespace ratio
{

class hyper_flaw : public flaw
{
  friend class solver;

public:
  hyper_flaw(solver &slv, resolver *const cause, const std::vector<flaw *> &fs);
  hyper_flaw(const hyper_flaw &orig) = delete;
  virtual ~hyper_flaw();

#ifdef BUILD_GUI
  std::string get_label() const override;
#endif

private:
  void compute_resolvers() override;

  class hyper_resolver : public resolver
  {
  public:
    hyper_resolver(solver &slv, hyper_flaw &s_flaw, const smt::var &app_r, const smt::rational &cst, const std::vector<resolver *> &rs);
    hyper_resolver(const hyper_resolver &that) = delete;
    virtual ~hyper_resolver();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

  private:
    void apply() override;

  private:
    const std::vector<resolver *> resolvers;
  };

private:
  const std::vector<flaw *> flaws;
};
} // namespace ratio
