#pragma once

#include "scope.h"

namespace ratio
{

class context;
class expr;
class item;

class method : public scope
{
public:
  method(core &cr, scope &scp, const type *const return_type, const std::string &name, const std::vector<field *> &args);
  method(const method &orig) = delete;
  virtual ~method();

  const type *get_type() const { return return_type; }         // returns the return type of this method (can be nullptr)..
  std::string get_name() const { return name; }                // returns the name of this method..
  const std::vector<field *> get_args() const { return args; } // returns the list of arguments of this method..

  item *invoke(context &ctx, const std::vector<expr> &exprs) const;

private:
  const type *const return_type; // the return type of this method (can be nullptr)..
  const std::string name;        // the name of this method..

protected:
  const std::vector<field *> args; // the arguments of this paper..
};
} // namespace ratio
