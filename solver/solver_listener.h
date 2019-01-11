#pragma once

#include "solver.h"
#include "graph.h"
#include "sat_value_listener.h"

namespace ratio
{

class solver_listener
{
  friend class solver;

public:
  solver_listener(solver &s) : slv(slv) { slv.listeners.push_back(this); }
  solver_listener(const solver_listener &orig) = delete;
  virtual ~solver_listener() { slv.listeners.erase(std::find(slv.listeners.begin(), slv.listeners.end(), this)); }

private:
  void new_flaw(const flaw &f)
  {
    flaw_listeners.insert({&f, new flaw_listener(*this, f)});
    flaw_created(f);
  }

  virtual void flaw_created(const flaw &f) {}
  virtual void flaw_state_changed(const flaw &f) {}
  virtual void current_flaw(const flaw &f) {}

  void new_resolver(const resolver &r)
  {
    resolver_listeners.insert({&r, new resolver_listener(*this, r)});
    resolver_created(r);
  }

  virtual void resolver_created(const resolver &r) {}
  virtual void resolver_state_changed(const resolver &r) {}
  virtual void resolver_cost_changed(const resolver &r) {}
  virtual void current_resolver(const resolver &r) {}

  virtual void causal_link_added(const flaw &f, const resolver &r) {}

  virtual void solution_found() {}

  class flaw_listener : public smt::sat_value_listener
  {
  public:
    flaw_listener(solver_listener &l, const flaw &f) : sat_value_listener(listener.slv.get_sat_core()), listener(listener), f(f) { listen_sat(f.get_phi()); }
    flaw_listener(const flaw_listener &orig) = delete;
    virtual ~flaw_listener() {}

  private:
    void sat_value_change(const smt::var &v) override { listener.flaw_state_changed(f); }

  protected:
    solver_listener &listener;
    const flaw &f;
  };

  class resolver_listener : public smt::sat_value_listener
  {
  public:
    resolver_listener(solver_listener &l, const resolver &r) : sat_value_listener(listener.slv.get_sat_core()), listener(listener), r(r) { listen_sat(r.get_rho()); }
    resolver_listener(const resolver_listener &orig) = delete;
    virtual ~resolver_listener() {}

  private:
    void sat_value_change(const smt::var &v) override { listener.resolver_state_changed(r); }

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