#pragma once

#include "type.h"

namespace ratio
{

class expr;

namespace ast
{
class expression;
}

class typedef_type : public type
{
  public:
    typedef_type(core &cr, scope &scp, const std::string &name, const type &base_type, const ast::expression *const e);
    typedef_type(const typedef_type &orig) = delete;
    virtual ~typedef_type();

    expr new_instance(context &ctx) override;

  private:
    const type &base_type;
    const ast::expression *const xpr;
};
}