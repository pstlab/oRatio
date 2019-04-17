#pragma once

#include "smart_type.h"
#include "constructor.h"
#include "graph.h"

#define PROPOSITIONAL_AGENT_NAME "PropositionalAgent"

namespace ratio
{

class propositional_agent : public smart_type
{
public:
  propositional_agent(solver &slv);
  propositional_agent(const propositional_agent &orig) = delete;
  virtual ~propositional_agent();

private:
  std::vector<std::vector<std::pair<smt::lit, double>>> get_current_incs() override;

  void new_fact(atom_flaw &f) override;
  void new_goal(atom_flaw &f) override;

  class agnt_constructor : public constructor
  {
  public:
    agnt_constructor(propositional_agent &agnt);
    agnt_constructor(agnt_constructor &&) = delete;
    virtual ~agnt_constructor();
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

private:
  std::set<atom *> to_check;
  std::vector<std::pair<atom *, agnt_atom_listener *>> atoms;
};
} // namespace ratio