#pragma once

#include "type.h"

namespace ratio
{
  class item;

  namespace ast
  {
    class enum_declaration;
  } // namespace ast

  class enum_type : public type
  {
    friend class ast::enum_declaration;

  public:
    enum_type(core &cr, scope &scp, std::string name);
    enum_type(const enum_type &orig) = delete;
    virtual ~enum_type();

    expr new_instance(context &ctx) override;

  private:
    std::vector<item *> get_all_instances() const noexcept;

  private:
    std::vector<enum_type *> enums;
  };
} // namespace ratio