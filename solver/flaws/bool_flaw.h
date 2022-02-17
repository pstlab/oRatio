#pragma once

#include "flaw.h"
#include "resolver.h"

namespace ratio
{
  class bool_item;

  class bool_flaw final : public flaw
  {
  public:
    bool_flaw(solver &slv, std::vector<resolver *> causes, bool_item &b_itm);
    bool_flaw(const bool_flaw &orig) = delete;

    std::string get_data() const noexcept override;

  private:
    void compute_resolvers() override;

    class choose_value final : public resolver
    {
    public:
      choose_value(solver &slv, smt::rational cst, bool_flaw &bl_flaw, const smt::lit &val);
      choose_value(const choose_value &that) = delete;

      std::string get_data() const noexcept override;

    private:
      void apply() override;
    };

  private:
    bool_item &b_itm; // the bool variable whose value has to be decided..
  };
} // namespace ratio