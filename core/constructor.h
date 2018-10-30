#pragma once

#include "scope.h"

namespace ratio
{

class expr;
class context;
class item;

class constructor : public scope
{
  friend class type;

public:
  constructor(core &cr, scope &scp, const std::vector<field *> &args);
  constructor(const constructor &orig) = delete;
  virtual ~constructor();

  const std::vector<field *> get_args() const { return args; } // returns the list of arguments of this constructor..

  expr new_instance(context &ctx, const std::vector<expr> &exprs) const; // creates a new instance of an item whose type has this constructor invoking this constructor within the given context with the given expressions as arguments of the constructor..

private:
  void invoke(item &i, const std::vector<expr> &exprs) const;

private:
  const std::vector<field *> args;
};
} // namespace ratio
