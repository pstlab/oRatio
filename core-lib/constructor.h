#pragma once

#include "scope.h"

namespace ratio
{

class expr;
class context;
class item;

namespace ast
{
class expression;
class statement;
}

class constructor : public scope
{
  friend class type;

public:
  constructor(core &cr, scope &scp, const std::vector<field *> &args, const std::vector<std::pair<std::string, std::vector<ast::expression *>>> &il, const std::vector<ast::statement *> &stmnts);
  constructor(const constructor &orig) = delete;
  virtual ~constructor();

  expr new_instance(context &ctx, const std::vector<expr> &exprs);

private:
  void invoke(item &i, const std::vector<expr> &exprs);

private:
  const std::vector<field *> args;
  const std::vector<std::pair<std::string, std::vector<ast::expression *>>> init_list;
  const std::vector<ast::statement *> statements;
};
}