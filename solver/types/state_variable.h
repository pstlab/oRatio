#pragma once

#include "smart_type.h"
#include "constructor.h"
#include "flaw.h"
#include "resolver.h"

#define STATE_VARIABLE_NAME "StateVariable"

namespace ratio
{
  class order_resolver;

  class state_variable : public smart_type
  {
    friend class order_resolver;

  public:
    state_variable(solver &slv);
    state_variable(const state_variable &orig) = delete;
    virtual ~state_variable();

  private:
    std::vector<std::vector<std::pair<smt::lit, double>>> get_current_incs() override;

    void new_predicate(predicate &pred) noexcept override;
    void new_atom(atom_flaw &f) override;
    void store_variables(atom &atm0, atom &atm1);

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
      void rdl_value_change(const smt::var &) override { something_changed(); }
      void lra_value_change(const smt::var &) override { something_changed(); }
      void ov_value_change(const smt::var &) override { something_changed(); }

    protected:
      state_variable &sv;
    };

    // the flaw (i.e. two or more temporally overlapping atoms on the same state-variable instance) that can arise from a state-variable..
    class sv_flaw : public flaw
    {
      friend class state_variable;

    public:
      sv_flaw(state_variable &sv, const std::set<atom *> &atms);
      sv_flaw(sv_flaw &&) = delete;
      virtual ~sv_flaw();

      std::string get_label() const override;

    private:
      void compute_resolvers() override;

    private:
      state_variable &sv;
      const std::set<atom *> overlapping_atoms;
    };

    // a resolver for temporally ordering atoms..
    class order_resolver : public resolver
    {
    public:
      order_resolver(sv_flaw &flw, const smt::lit &r, const atom &before, const atom &after);
      order_resolver(const order_resolver &that) = delete;
      virtual ~order_resolver();

      std::string get_label() const override;

    private:
      void apply() override;

    private:
      const atom &before; // applying the resolver will order this atom before the other..
      const atom &after;  // applying the resolver will order this atom after the other..
    };

    // a resolver for placing atoms on a specific state-variable..
    class place_resolver : public resolver
    {
    public:
      place_resolver(sv_flaw &flw, const smt::lit &r, atom &plc_atm, const item &plc_itm, atom &frbd_atm);
      place_resolver(const place_resolver &that) = delete;
      virtual ~place_resolver();

      std::string get_label() const override;

    private:
      void apply() override;

    private:
      atom &plc_atm;       // applying the resolver will force this atom on the 'plc_item' item..
      const item &plc_itm; // applying the resolver will force the 'plc_atm' atom on this item..
      atom &frbd_atm;      // applying the resolver will forbid this atom on the 'plc_itm' item..
    };

    // a resolver for forbidding atoms on a specific state-variable..
    class forbid_resolver : public resolver
    {
    public:
      forbid_resolver(sv_flaw &flw, atom &atm, item &itm);
      forbid_resolver(const forbid_resolver &that) = delete;
      virtual ~forbid_resolver();

      std::string get_label() const override;

    private:
      void apply() override;

    private:
      atom &atm; // applying the resolver will forbid this atom on the 'itm' item..
      item &itm; // applying the resolver will forbid the 'atm' atom on this item..
    };

  private:
    const predicate &int_pred;
    std::set<const item *> to_check;                          // the state-variable instances whose atoms have changed..
    std::vector<std::pair<atom *, sv_atom_listener *>> atoms; // we store, for each atom, its atom listener..

    std::map<std::set<atom *>, sv_flaw *> sv_flaws;                                  // the state-variable flaws found so far..
    std::map<atom *, std::map<atom *, smt::lit>> leqs;                               // all the possible ordering constraints..
    std::map<std::set<atom *>, std::vector<std::pair<smt::lit, const item *>>> plcs; // all the possible placement constraints..
  };
} // namespace ratio