#pragma once

#include "smart_type.h"
#include "constructor.h"
#include "graph.h"

#define STATE_VARIABLE_NAME "StateVariable"

namespace ratio
{

class state_variable : public smart_type
{
public:
  state_variable(solver &slv);
  state_variable(const state_variable &orig) = delete;
  virtual ~state_variable();

private:
  std::vector<flaw *> get_flaws() override;

  void new_predicate(predicate &pred) override;
  void new_fact(atom_flaw &f) override;
  void new_goal(atom_flaw &f) override;

  // the state-variable constructor..
  class sv_constructor : public constructor
  {
  public:
    sv_constructor(state_variable &sv);
    sv_constructor(sv_constructor &&) = delete;
    virtual ~sv_constructor();
  };

  // the atom listener for the state-variable..
  class sv_atom_listener : public atom_listener
  {
  public:
    sv_atom_listener(state_variable &sv, atom &atm);
    sv_atom_listener(sv_atom_listener &&) = delete;
    virtual ~sv_atom_listener();

  private:
    void something_changed();

    void sat_value_change(const smt::var &) override { something_changed(); }
    void lra_value_change(const smt::var &) override { something_changed(); }
    void ov_value_change(const smt::var &) override { something_changed(); }

  protected:
    state_variable &sv;
  };

  // the flaw (i.e. two or more temporally overlapping atoms on the same state-variable instance) that can arise from a state-variable..
  class sv_flaw : public flaw
  {
  public:
    sv_flaw(solver &slv, const std::set<atom *> &overlapping_atoms);
    sv_flaw(sv_flaw &&) = delete;
    virtual ~sv_flaw();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

  private:
    void compute_resolvers() override;

  private:
    const std::set<atom *> overlapping_atoms;
  };

  // a resolver for temporally ordering atoms..
  class order_resolver : public resolver
  {
  public:
    order_resolver(solver &slv, const smt::var &r, sv_flaw &f, const atom &before, const atom &after);
    order_resolver(const order_resolver &that) = delete;
    virtual ~order_resolver();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

  private:
    void apply() override;

  private:
    const atom &before;
    const atom &after;
  };

  // a resolver for displacing atoms..
  class displace_resolver : public resolver
  {
  public:
    displace_resolver(solver &slv, sv_flaw &f, const atom &a0, const atom &a1, const smt::lit &neq_lit);
    displace_resolver(const displace_resolver &that) = delete;
    virtual ~displace_resolver();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

  private:
    void apply() override;

  private:
    const atom &a0;
    const atom &a1;
    const smt::lit neq_lit;
  };

private:
  std::set<item *> to_check;                                // the state-variable instances whose atoms have changed..
  std::vector<std::pair<atom *, sv_atom_listener *>> atoms; // we store, for each atom, its atom listener..
};
} // namespace ratio
