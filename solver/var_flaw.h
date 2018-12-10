#pragma once

#include "graph.h"

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
  var_flaw(solver &slv, resolver *const cause, var_item &v_itm);
  var_flaw(const var_flaw &orig) = delete;
  virtual ~var_flaw();

#ifdef BUILD_GUI
  std::string get_label() const override;
#endif

private:
  void compute_resolvers() override;

  class choose_value : public resolver
  {
  public:
    choose_value(solver &slv, smt::rational cst, var_flaw &enm_flaw, smt::var_value &val);
    choose_value(const choose_value &that) = delete;
    virtual ~choose_value();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

  private:
    void apply() override;

  private:
    smt::var v;
    smt::var_value &val;
  };

private:
  var_item &v_itm;
};
} // namespace ratio