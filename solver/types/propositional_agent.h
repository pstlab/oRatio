#pragma once

#include "smart_type.h"
#include "constructor.h"
#include "flaw.h"
#include "resolver.h"

#define PROPOSITIONAL_AGENT_NAME "PropositionalAgent"

namespace ratio
{

class propositional_agent : public smart_type
{
public:
  propositional_agent(solver &s);
  propositional_agent(const propositional_agent &orig) = delete;
  virtual ~propositional_agent();

private:
  std::vector<flaw *> get_flaws() override;

  void new_fact(atom_flaw &f) override;
  void new_goal(atom_flaw &f) override;

  class agnt_constructor : public constructor
  {
  public:
    agnt_constructor(propositional_agent &agnt) : constructor(agnt.slv, agnt, {}, {}, {}) {}
    agnt_constructor(agnt_constructor &&) = delete;
    virtual ~agnt_constructor() {}
  };

  class agnt_atom_listener : public atom_listener
  {
  public:
    agnt_atom_listener(propositional_agent &agnt, atom &a);
    agnt_atom_listener(agnt_atom_listener &&) = delete;
    virtual ~agnt_atom_listener();

  private:
    void something_changed();

    void sat_value_change(const smt::var &) override { something_changed(); }
    void lra_value_change(const smt::var &) override { something_changed(); }
    void ov_value_change(const smt::var &) override { something_changed(); }

  protected:
    propositional_agent &agnt;
  };

  class agnt_flaw : public flaw
  {
  public:
    agnt_flaw(solver &s, const std::set<atom *> &overlapping_atoms);
    agnt_flaw(agnt_flaw &&) = delete;
    virtual ~agnt_flaw();

    std::string get_label() const override { return "agent-flaw"; }

  private:
    void compute_resolvers() override;

  private:
    const std::set<atom *> overlapping_atoms;
  };

  class order_resolver : public resolver
  {
  public:
    order_resolver(solver &slv, const smt::var &r, agnt_flaw &f, const atom &before, const atom &after);
    order_resolver(const order_resolver &that) = delete;
    virtual ~order_resolver();

    std::string get_label() const override { return "ρ" + std::to_string(rho) + " σ" + std::to_string(before.sigma) + " <= σ" + std::to_string(after.sigma); }

  private:
    void apply() override;

  private:
    const atom &before;
    const atom &after;
  };

  class displace_resolver : public resolver
  {
  public:
    displace_resolver(solver &slv, agnt_flaw &f, const atom &a0, const atom &a1, const smt::lit &neq_lit);
    displace_resolver(const displace_resolver &that) = delete;
    virtual ~displace_resolver();

    std::string get_label() const override { return "ρ" + std::to_string(rho) + " displ σ" + std::to_string(a0.sigma) + ".τ != σ" + std::to_string(a1.sigma) + ".τ"; }

  private:
    void apply() override;

  private:
    const atom &a0;
    const atom &a1;
    const smt::lit neq_lit;
  };

private:
  std::set<atom *> to_check;
  std::vector<std::pair<atom *, agnt_atom_listener *>> atoms;
};
}