#pragma once

#include "scope.h"

namespace riddle::ast
{
  class expression;
  class statement;
} // namespace riddle::ast

namespace ratio
{
  class expr;
  class context;
  class item;

  class constructor : public scope
  {
    friend class type;

  public:
    CORE_EXPORT constructor(core &cr, scope &scp, const std::vector<const field *> &args, const std::vector<std::pair<const std::string, const std::vector<const riddle::ast::expression *>>> &il, const std::vector<const riddle::ast::statement *> &stmnts);
    constructor(const constructor &orig) = delete;
    CORE_EXPORT virtual ~constructor();

    inline const std::vector<const field *> get_args() const noexcept { return args; } // returns the list of arguments of this constructor..

    expr new_instance(context &ctx, const std::vector<expr> &exprs) const noexcept; // creates a new instance of an item whose type has this constructor invoking this constructor within the given context with the given expressions as arguments of the constructor..

  private:
    void invoke(item &i, const std::vector<expr> &exprs) const;

  private:
    const std::vector<const field *> args;
    const std::vector<std::pair<const std::string, const std::vector<const riddle::ast::expression *>>> init_list;
    const std::vector<const riddle::ast::statement *> statements;
  };
} // namespace ratio