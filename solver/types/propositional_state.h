#pragma once

#include "smart_type.h"
#include "constructor.h"
#include "predicate.h"

#define PROPOSITIONAL_STATE_NAME "PropositionalState"
#define PROPOSITIONAL_STATE_PREDICATE_NAME "PropositionalStatePredicate"
#define PROPOSITIONAL_STATE_POLARITY_NAME "polarity"

namespace ratio
{
  class propositional_state : public smart_type
  {
  public:
    propositional_state(solver &slv);
    propositional_state(const propositional_state &orig) = delete;
    virtual ~propositional_state();

  private:
    std::vector<std::vector<std::pair<smt::lit, double>>> get_current_incs() override;

    void new_predicate(predicate &pred) noexcept override;
    void new_atom(atom_flaw &f) override;

    // the propositional-state constructor..
    class ps_constructor : public constructor
    {
    public:
      ps_constructor(propositional_state &ps);
      ps_constructor(ps_constructor &&) = delete;
      virtual ~ps_constructor();
    };

    // the propositional-state predicate..
    class ps_predicate : public predicate
    {
    public:
      ps_predicate(propositional_state &rr);
      ps_predicate(ps_predicate &&) = delete;
      virtual ~ps_predicate();
    };

    // the atom listener for propositional-state..
    class ps_atom_listener : public atom_listener
    {
    public:
      ps_atom_listener(propositional_state &ps, atom &a);
      ps_atom_listener(ps_atom_listener &&) = delete;
      virtual ~ps_atom_listener();

    private:
      void something_changed();

      void sat_value_change(const smt::var &) override { something_changed(); }
      void rdl_value_change(const smt::var &) override { something_changed(); }
      void lra_value_change(const smt::var &) override { something_changed(); }
      void ov_value_change(const smt::var &) override { something_changed(); }

    protected:
      propositional_state &ps;
    };

  private:
    const predicate &int_pred;
    std::set<atom *> to_check;
    std::vector<std::pair<atom *, ps_atom_listener *>> atoms;
  };
} // namespace ratio