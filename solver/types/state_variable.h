#pragma once

#include "smart_type.h"
#include "constructor.h"

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
  std::vector<std::vector<std::pair<smt::lit, double>>> get_current_incs() override;

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

private:
  std::set<item *> to_check;                                // the state-variable instances whose atoms have changed..
  std::vector<std::pair<atom *, sv_atom_listener *>> atoms; // we store, for each atom, its atom listener..
};
} // namespace ratio
