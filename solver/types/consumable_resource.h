#pragma once

#include "smart_type.h"
#include "timelines_extractor.h"
#include "constructor.h"
#include "predicate.h"
#include "flaw.h"
#include "resolver.h"

#define CONSUMABLE_RESOURCE_NAME "ConsumableResource"
#define CONSUMABLE_RESOURCE_INITIAL_AMOUNT "initial_amount"
#define CONSUMABLE_RESOURCE_CAPACITY "capacity"
#define CONSUMABLE_RESOURCE_PRODUCE_PREDICATE_NAME "Produce"
#define CONSUMABLE_RESOURCE_CONSUME_PREDICATE_NAME "Consume"
#define CONSUMABLE_RESOURCE_USE_AMOUNT_NAME "amount"

namespace ratio
{
  class consumable_resource final : public smart_type, public timelines_extractor
  {
  public:
    consumable_resource(solver &slv);
    consumable_resource(const consumable_resource &orig) = delete;
    ~consumable_resource();

  private:
    std::vector<std::vector<std::pair<smt::lit, double>>> get_current_incs() override;

    // the consumable-resource constructor..
    class cr_constructor final : public constructor
    {
    public:
      cr_constructor(consumable_resource &rr);
      cr_constructor(cr_constructor &&) = delete;
    };

    // the consumable-resource 'Produce' predicate..
    class produce_predicate final : public predicate
    {
    public:
      produce_predicate(consumable_resource &rr);
      produce_predicate(produce_predicate &&) = delete;
    };

    // the consumable-resource 'Consume' predicate..
    class consume_predicate final : public predicate
    {
    public:
      consume_predicate(consumable_resource &rr);
      consume_predicate(consume_predicate &&) = delete;
    };

    // the atom listener for the consumable-resource..
    class cr_atom_listener final : public atom_listener
    {
    public:
      cr_atom_listener(consumable_resource &rr, atom &atm);
      cr_atom_listener(cr_atom_listener &&) = delete;

    private:
      void something_changed();

      void sat_value_change(const smt::var &) override { something_changed(); }
      void rdl_value_change(const smt::var &) override { something_changed(); }
      void lra_value_change(const smt::var &) override { something_changed(); }
      void ov_value_change(const smt::var &) override { something_changed(); }

    protected:
      consumable_resource &cr;
    };

  public:
    smt::json extract_timelines() const noexcept override;

  private:
    std::set<const item *> to_check;                          // the consumable-resource instances whose atoms have changed..
    std::vector<std::pair<atom *, cr_atom_listener *>> atoms; // we store, for each atom, its atom listener..
  };
} // namespace ratio
