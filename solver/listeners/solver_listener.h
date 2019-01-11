#pragma once

#include "sat_value_listener.h"

namespace ratio
{

class solver;
class flaw;
class resolver;

class solver_listener
{
  friend void fire_new_flaw(const solver &s, const flaw &f);
  friend void fire_flaw_state_changed(const solver &s, const flaw &f);
  friend void fire_current_flaw(const solver &s, const flaw &f);
  friend void fire_new_resolver(const solver &s, const resolver &r);
  friend void fire_resolver_state_changed(const solver &s, const resolver &r);
  friend void fire_resolver_cost_changed(const solver &s, const resolver &r);
  friend void fire_current_resolver(const solver &s, const resolver &r);
  friend void fire_causal_link_added(const solver &s, const flaw &f, const resolver &r);
  friend void fire_solution_found(const solver &s);

public:
  solver_listener(solver &s);
  solver_listener(const solver_listener &orig) = delete;
  virtual ~solver_listener();

private:
  void new_flaw(const flaw &f);

  virtual void flaw_created(const flaw &f);
  virtual void flaw_state_changed(const flaw &f);
  virtual void current_flaw(const flaw &f);

  void new_resolver(const resolver &r);

  virtual void resolver_created(const resolver &r);
  virtual void resolver_state_changed(const resolver &r);
  virtual void resolver_cost_changed(const resolver &r);
  virtual void current_resolver(const resolver &r);

  virtual void causal_link_added(const flaw &f, const resolver &r);

  virtual void solution_found();

  class flaw_listener : public smt::sat_value_listener
  {
  public:
    flaw_listener(solver_listener &l, const flaw &f);
    flaw_listener(const flaw_listener &orig) = delete;
    virtual ~flaw_listener();

  private:
    void sat_value_change(const smt::var &v) override;

  protected:
    solver_listener &listener;
    const flaw &f;
  };

  class resolver_listener : public smt::sat_value_listener
  {
  public:
    resolver_listener(solver_listener &l, const resolver &r);
    resolver_listener(const resolver_listener &orig) = delete;
    virtual ~resolver_listener();

  private:
    void sat_value_change(const smt::var &v) override;

  protected:
    solver_listener &listener;
    const resolver &r;
  };

protected:
  solver &slv;

private:
  std::unordered_map<const flaw *, flaw_listener *> flaw_listeners;
  std::unordered_map<const resolver *, resolver_listener *> resolver_listeners;
};
} // namespace ratio