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
  constructor(core &cr, scope &scp, const std::vector<field *> &args, const std::vector<std::pair<std::string, std::vector<riddle::ast::expression *>>> &il, const std::vector<riddle::ast::statement *> &stmnts);
  constructor(const constructor &orig) = delete;
  virtual ~constructor();

  const std::vector<field *> get_args() const { return args; } // returns the list of arguments of this constructor..

  expr new_instance(context &ctx, const std::vector<expr> &exprs) const; // creates a new instance of an item whose type has this constructor invoking this constructor within the given context with the given expressions as arguments of the constructor..

private:
  void invoke(item &i, const std::vector<expr> &exprs) const;

private:
  const std::vector<field *> args;
  const std::vector<std::pair<std::string, std::vector<riddle::ast::expression *>>> init_list;
  const std::vector<riddle::ast::statement *> statements;
};
} // namespace ratio
