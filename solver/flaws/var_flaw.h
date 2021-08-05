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

  class var_flaw final : public flaw
  {
  public:
    var_flaw(solver &slv, std::vector<resolver *> causes, var_item &v_itm);
    var_flaw(const var_flaw &orig) = delete;

    std::string get_label() const noexcept override;

  private:
    void compute_resolvers() override;

    class choose_value final : public resolver
    {
    public:
      choose_value(solver &slv, smt::rational cst, var_flaw &enm_flaw,  smt::var_value &val);
      choose_value(const choose_value &that) = delete;

      std::string get_label() const noexcept override;

    private:
      void apply() override;

    private:
      smt::var v;                // the object variable whose value has to be decided..
       smt::var_value &val; // the decided value..
    };

  private:
    var_item &v_itm; // the enum variable whose value has to be decided..
  };
} // namespace ratio