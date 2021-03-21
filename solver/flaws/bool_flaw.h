#pragma once

#include "flaw.h"
#include "resolver.h"

namespace ratio
{
  class bool_item;

  class bool_flaw : public flaw
  {
  public:
    bool_flaw(solver &slv, const std::vector<resolver *> &causes, bool_item &b_itm);
    bool_flaw(const bool_flaw &orig) = delete;
    virtual ~bool_flaw();

    std::string get_label() const noexcept override;

  private:
    void compute_resolvers() override;

    class choose_value : public resolver
    {
    public:
      choose_value(solver &slv, smt::rational cst, bool_flaw &bl_flaw, const smt::lit &val);
      choose_value(const choose_value &that) = delete;
      virtual ~choose_value();

      std::string get_label() const noexcept override;

    private:
      void apply() override;
    };

  private:
    bool_item &b_itm; // the bool variable whose value has to be decided..
  };
} // namespace ratio