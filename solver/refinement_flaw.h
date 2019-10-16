#pragma once

#include "flaw.h"
#include "resolver.h"

namespace ratio
{

class refinement_flaw : public flaw
{
  friend class graph;

public:
  refinement_flaw(graph &gr, resolver *const cause, const std::vector<flaw *> &fs);
  refinement_flaw(const refinement_flaw &orig) = delete;
  virtual ~refinement_flaw();

#ifdef BUILD_GUI
  std::string get_label() const override;
#endif

private:
  void compute_resolvers() override;

  class refinement_resolver : public resolver
  {
  public:
    refinement_resolver(graph &gr, refinement_flaw &s_flaw, const std::vector<resolver *> &rs);
    refinement_resolver(const refinement_resolver &that) = delete;
    virtual ~refinement_resolver();

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