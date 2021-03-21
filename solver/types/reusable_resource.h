#pragma once

#include "smart_type.h"
#include "constructor.h"
#include "predicate.h"
#include "flaw.h"
#include "resolver.h"

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

    void new_predicate(predicate &) noexcept override;
    void new_atom(atom_flaw &f) override;
    void store_variables(atom &atm0, atom &atm1);

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
      void rdl_value_change(const smt::var &) override { something_changed(); }
      void lra_value_change(const smt::var &) override { something_changed(); }
      void ov_value_change(const smt::var &) override { something_changed(); }

    protected:
      reusable_resource &rr;
    };

    // the flaw (i.e. two or more temporally overlapping atoms on the same reusable-resource instance) that can arise from a reusable-resource..
    class rr_flaw : public flaw
    {
      friend class state_variable;

    public:
      rr_flaw(reusable_resource &rr, const std::set<atom *> &atms);
      rr_flaw(rr_flaw &&) = delete;
      virtual ~rr_flaw();

      std::string get_label() const override;

    private:
      void compute_resolvers() override;

    private:
      reusable_resource &rr;
      const std::set<atom *> overlapping_atoms;
    };

    // a resolver for temporally ordering atoms..
    class order_resolver : public resolver
    {
    public:
      order_resolver(rr_flaw &flw, const smt::lit &r, const atom &before, const atom &after);
      order_resolver(const order_resolver &that) = delete;
      virtual ~order_resolver();

      std::string get_label() const override;

    private:
      void apply() override;

    private:
      const atom &before; // applying the resolver will order this atom before the other..
      const atom &after;  // applying the resolver will order this atom after the other..
    };

    // a resolver for placing atoms on a specific reusable-resource..
    class place_resolver : public resolver
    {
    public:
      place_resolver(rr_flaw &flw, const smt::lit &r, atom &plc_atm, const item &plc_itm, atom &frbd_atm);
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

    // a resolver for forbidding atoms on a specific reusable-resource..
    class forbid_resolver : public resolver
    {
    public:
      forbid_resolver(rr_flaw &flw, atom &atm, item &itm);
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
    std::set<const item *> to_check;                          // the reusable-resource instances whose atoms have changed..
    std::vector<std::pair<atom *, rr_atom_listener *>> atoms; // we store, for each atom, its atom listener..

    std::map<std::set<atom *>, rr_flaw *> rr_flaws;                                  // the reusable-resource flaws found so far..
    std::map<atom *, std::map<atom *, smt::lit>> leqs;                               // all the possible ordering constraints..
    std::map<std::set<atom *>, std::vector<std::pair<smt::lit, const item *>>> plcs; // all the possible placement constraints..
  };
} // namespace ratio