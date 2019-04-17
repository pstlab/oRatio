#pragma once

#include "smart_type.h"
#include "constructor.h"
#include "predicate.h"
#include "graph.h"

#define REUSABLE_RESOURCE_NAME "ReusableResource"
#define REUSABLE_RESOURCE_CAPACITY "capacity"
#define REUSABLE_RESOURCE_USE_PREDICATE_NAME "Use"
#define REUSABLE_RESOURCE_USE_AMOUNT_NAME "amount"

namespace ratio
{

class reusable_resource : public smart_type
{
public:
  reusable_resource(solver &slv);
  reusable_resource(const reusable_resource &orig) = delete;
  virtual ~reusable_resource();

private:
  std::vector<std::vector<std::pair<smt::lit, double>>> get_current_incs() override;

  void new_predicate(predicate &) override;
  void new_fact(atom_flaw &f) override;
  void new_goal(atom_flaw &f) override;

  // the reusable-resource constructor..
  class rr_constructor : public constructor
  {
  public:
    rr_constructor(reusable_resource &rr);
    rr_constructor(rr_constructor &&) = delete;
    virtual ~rr_constructor();
  };

  // the reusable-resource 'use' predicate..
  class use_predicate : public predicate
  {
  public:
    use_predicate(reusable_resource &rr);
    use_predicate(use_predicate &&) = delete;
    virtual ~use_predicate();
  };

  // the atom listener for the reusable-resource..
  class rr_atom_listener : public atom_listener
  {
  public:
    rr_atom_listener(reusable_resource &rr, atom &atm);
    rr_atom_listener(rr_atom_listener &&) = delete;
    virtual ~rr_atom_listener();

  private:
    void something_changed();

    void sat_value_change(const smt::var &) override { something_changed(); }
    void lra_value_change(const smt::var &) override { something_changed(); }
    void ov_value_change(const smt::var &) override { something_changed(); }

  protected:
    reusable_resource &rr;
  };

private:
  std::set<item *> to_check;
  std::vector<std::pair<atom *, rr_atom_listener *>> atoms;
};
} // namespace ratio