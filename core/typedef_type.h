#pragma once

#include "type.h"

namespace riddle::ast
{
  class expression;
} // namespace riddle::ast

namespace ratio
{
  class typedef_type final : public type
  {
  public:
    typedef_type(core &cr, scope &scp, const std::string &name, const type &base_type, const riddle::ast::expression *const e);
    typedef_type(const typedef_type &orig) = delete;

    expr new_instance(context &ctx) override;

  private:
    const type &base_type;
    const riddle::ast::expression *const xpr;
  };
} // namespace ratio