#pragma once

#include "scope.h"

namespace riddle::ast
{
  class statement;
} // namespace riddle::ast

namespace ratio
{
  class context;
  class expr;
  class item;

  class method : public scope
  {
    friend class core;
    friend class type;

  public:
    method(core &cr, scope &scp, const type *const return_type, const std::string &name, const std::vector<const field *> &args, const std::vector<const riddle::ast::statement *> &stmnts);
    method(const method &orig) = delete;
    virtual ~method();

    inline const type *get_return_type() const noexcept { return return_type; }        // returns the return type of this method (can be nullptr)..
    inline std::string get_name() const noexcept { return name; }                      // returns the name of this method..
    inline const std::vector<const field *> get_args() const noexcept { return args; } // returns the list of arguments of this method..

    item *invoke(context &ctx, const std::vector<expr> &exprs) const;

  private:
    const type *const return_type; // the return type of this method (can be nullptr)..
    const std::string name;        // the name of this method..

  private:
    const std::vector<const field *> args;                        // the arguments of this paper..
    const std::vector<const riddle::ast::statement *> statements; // the statements within the method's body..
  };
} // namespace ratio