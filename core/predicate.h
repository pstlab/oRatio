#pragma once

#include "type.h"

namespace ratio
{

class expr;
class atom;

class predicate : public type
{
public:
  predicate(core &cr, scope &scp, const std::string &name, const std::vector<field *> &args);
  predicate(const predicate &orig) = delete;
  virtual ~predicate();

  const std::vector<field *> get_args() const { return args; }

  expr new_instance(context &ctx) override; // creates a new atom having this predicate within the given context..

  void apply_rule(atom &a) const; // applies the rule associated to this predicate to the given atom..

protected:
  const std::vector<field *> args;
};
} // namespace ratio