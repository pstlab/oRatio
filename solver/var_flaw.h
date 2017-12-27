#pragma once

#include "flaw.h"
#include "item.h"
#include "resolver.h"

namespace ratio
{

class var_flaw : public flaw
{
public:
  var_flaw(solver &slv, resolver *const cause, var_item &v_itm);
  var_flaw(const var_flaw &orig) = delete;
  virtual ~var_flaw();

  std::string get_label() const override { return "φ" + std::to_string(get_phi()) + " enum"; }

private:
  void compute_resolvers() override;

  class choose_value : public resolver
  {
  public:
    choose_value(solver &slv, smt::rational cst, var_flaw &enm_flaw, smt::var_value &val);
    choose_value(const choose_value &that) = delete;
    virtual ~choose_value();

    std::string get_label() const override { return "ρ" + std::to_string(rho) + " val"; }

  private:
    void apply() override;

  private:
    smt::var v;
    smt::var_value &val;
  };

private:
  var_item &v_itm;
};
}