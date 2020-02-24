#pragma once

#include "flaw.h"
#include "resolver.h"

namespace smt
{
class var_value;
} // namespace smt

namespace ratio
{

class var_item;

class var_flaw : public flaw
{
public:
  var_flaw(graph &gr, resolver *const cause, var_item &v_itm);
  var_flaw(const var_flaw &orig) = delete;
  virtual ~var_flaw();

  std::string get_label() const override;

private:
  void compute_resolvers() override;

  class choose_value : public resolver
  {
  public:
    choose_value(graph &gr, smt::rational cst, var_flaw &enm_flaw, smt::var_value &val);
    choose_value(const choose_value &that) = delete;
    virtual ~choose_value();

    std::string get_label() const override;

  private:
    void apply() override;

  private:
    smt::var v;          // the object variable whose value has to be decided..
    smt::var_value &val; // the decided value..
  };

private:
  var_item &v_itm; // the enum variable whose value has to be decided..
};
} // namespace ratio