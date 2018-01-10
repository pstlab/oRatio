#pragma once

#include "smart_type.h"
#include "constructor.h"
#include "predicate.h"
#include "flaw.h"
#include "resolver.h"

#define PROPOSITIONAL_STATE_NAME "PropositionalState"
#define PROPOSITIONAL_STATE_PREDICATE_NAME "PropositionalStatePredicate"
#define PROPOSITIONAL_STATE_POLARITY_NAME "polarity"

namespace ratio
{

class propositional_state : public smart_type
{
public:
  propositional_state(solver &s);
  propositional_state(const propositional_state &orig) = delete;
  virtual ~propositional_state();

private:
  std::vector<flaw *> get_flaws() override;

  void new_predicate(predicate &pred) override;
  void new_fact(atom_flaw &f) override;
  void new_goal(atom_flaw &f) override;

  class ps_constructor : public constructor
  {
  public:
    ps_constructor(propositional_state &ps) : constructor(ps.slv, ps, {}, {}, {}) {}
    ps_constructor(ps_constructor &&) = delete;
    virtual ~ps_constructor() {}
  };

  class ps_predicate : public predicate
  {
  public:
    ps_predicate(propositional_state &rr);
    ps_predicate(ps_predicate &&) = delete;
    virtual ~ps_predicate();
  };

  class ps_atom_listener : public atom_listener
  {
  public:
    ps_atom_listener(propositional_state &ps, atom &a);
    ps_atom_listener(ps_atom_listener &&) = delete;
    virtual ~ps_atom_listener();

  private:
    void something_changed();

    void sat_value_change(const smt::var &) override { something_changed(); }
    void lra_value_change(const smt::var &) override { something_changed(); }
    void ov_value_change(const smt::var &) override { something_changed(); }

  protected:
    propositional_state &ps;
  };

  class ps_flaw : public flaw
  {
  public:
    ps_flaw(solver &s, const std::set<atom *> &overlapping_atoms);
    ps_flaw(ps_flaw &&) = delete;
    virtual ~ps_flaw();

    std::string get_label() const override { return "φ" + std::to_string(get_phi()) + " ps-flaw"; }

  private:
    void compute_resolvers() override;

  private:
    const std::set<atom *> overlapping_atoms;
  };

  class order_resolver : public resolver
  {
  public:
    order_resolver(solver &slv, const smt::var &r, ps_flaw &f, const atom &before, const atom &after);
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
    displace_resolver(solver &slv, ps_flaw &f, const atom &a0, const atom &a1, const smt::lit &neq_lit);
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
  std::vector<std::pair<atom *, ps_atom_listener *>> atoms;
};
}