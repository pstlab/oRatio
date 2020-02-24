#pragma once

#include "flaw.h"
#include "resolver.h"

namespace ratio
{

class refinement_flaw : public flaw
{
  friend class graph;

public:
  refinement_flaw(graph &gr, resolver *const cause, flaw &to_enqueue, const std::vector<resolver *> &non_mtx_rs);
  refinement_flaw(const refinement_flaw &orig) = delete;
  virtual ~refinement_flaw();

  std::string get_label() const override;

private:
  void compute_resolvers() override;

  class refinement_resolver : public resolver
  {
  public:
    refinement_resolver(graph &gr, const smt::var &r, refinement_flaw &s_flaw, resolver &non_mtx_r);
    refinement_resolver(graph &gr, refinement_flaw &s_flaw, resolver &non_mtx_r);
    refinement_resolver(const refinement_resolver &that) = delete;
    virtual ~refinement_resolver();

    std::string get_label() const override;

  private:
    void apply() override;

  private:
    resolver &non_mtx_r; // the non-mutex resolver..
  };

private:
  flaw &to_enqueue;
  const std::vector<resolver *> non_mtx_rs;
};
} // namespace ratio