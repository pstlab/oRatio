#pragma once

#include "flaw.h"
#include "resolver.h"

namespace ratio
{

class mutex_flaw : public flaw
{
  friend class graph;

public:
  mutex_flaw(graph &gr, resolver *const cause, flaw &to_enqueue, const std::vector<resolver *> &non_mtx_rs);
  mutex_flaw(const mutex_flaw &orig) = delete;
  virtual ~mutex_flaw();

#ifdef BUILD_GUI
  std::string get_label() const override;
#endif

private:
  void compute_resolvers() override;

  class mutex_resolver : public resolver
  {
  public:
    mutex_resolver(graph &gr, const smt::var &r, mutex_flaw &s_flaw, resolver &non_mtx_r);
    mutex_resolver(graph &gr, mutex_flaw &s_flaw, resolver &non_mtx_r);
    mutex_resolver(const mutex_resolver &that) = delete;
    virtual ~mutex_resolver();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

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