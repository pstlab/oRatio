#pragma once

#include "flaw.h"
#include "resolver.h"

namespace ratio
{

class composite_flaw : public flaw
{
  friend class graph;

public:
  composite_flaw(graph &gr, resolver *const cause, const std::vector<flaw *> &fs);
  composite_flaw(const composite_flaw &orig) = delete;
  virtual ~composite_flaw();

#ifdef BUILD_GUI
  std::string get_label() const override;
#endif

private:
  void compute_resolvers() override;

  class composite_resolver : public resolver
  {
  public:
    composite_resolver(graph &gr, composite_flaw &s_flaw, const std::vector<resolver *> &rs);
    composite_resolver(const composite_resolver &that) = delete;
    virtual ~composite_resolver();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

  private:
    void apply() override;

  private:
    const std::vector<resolver *> resolvers; // the resolvers composing the composite resolver..
  };

private:
  const std::vector<flaw *> flaws;
};
} // namespace ratio