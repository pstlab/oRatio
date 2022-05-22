#pragma once

#include "solver.h"
#include "flaw.h"
#include "resolver.h"
#include "sat_value_listener.h"
#include "idl_value_listener.h"
#include <algorithm>

namespace ratio
{
  class solver_listener
  {
    friend class solver;

  public:
    solver_listener(solver &s) : slv(s) { slv.listeners.push_back(this); }
    solver_listener(const solver_listener &orig) = delete;
    virtual ~solver_listener() { slv.listeners.erase(std::find(slv.listeners.cbegin(), slv.listeners.cend(), this)); }

  private:
    void new_flaw(const flaw &f)
    {
      flaw_listeners.emplace(&f, new flaw_listener(*this, f));
      flaw_created(f);
    }

    virtual void flaw_created(const flaw &) {}
    virtual void flaw_state_changed(const flaw &) {}
    virtual void flaw_cost_changed(const flaw &) {}
    virtual void flaw_position_changed(const flaw &) {}
    virtual void current_flaw(const flaw &) {}

    void new_resolver(const resolver &r)
    {
      resolver_listeners.emplace(&r, new resolver_listener(*this, r));
      resolver_created(r);
    }

    virtual void resolver_created(const resolver &) {}
    virtual void resolver_state_changed(const resolver &) {}
    virtual void current_resolver(const resolver &) {}

    virtual void causal_link_added(const flaw &, const resolver &) {}

    class flaw_listener : public smt::sat_value_listener, public smt::idl_value_listener
    {
    public:
      flaw_listener(solver_listener &l, const flaw &f) : sat_value_listener(l.slv.get_sat_core()), idl_value_listener(l.slv.get_idl_theory()), listener(l), f(f)
      {
        listen_sat(variable(f.get_phi()));
        listen_idl(f.get_position());
      }
      flaw_listener(const flaw_listener &orig) = delete;
      virtual ~flaw_listener() {}

    private:
      void sat_value_change(const smt::var &) override { listener.flaw_state_changed(f); }
      void idl_value_change(const smt::var &) override { listener.flaw_position_changed(f); }

    protected:
      solver_listener &listener;
      const flaw &f;
    };

    class resolver_listener : public smt::sat_value_listener
    {
    public:
      resolver_listener(solver_listener &l, const resolver &r) : sat_value_listener(l.slv.get_sat_core()), listener(l), r(r) { listen_sat(variable(r.get_rho())); }
      resolver_listener(const resolver_listener &orig) = delete;
      virtual ~resolver_listener() {}

    private:
      void sat_value_change(const smt::var &) override { listener.resolver_state_changed(r); }

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