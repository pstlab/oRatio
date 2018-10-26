#pragma once

#include "type.h"

namespace ratio
{

class predicate : public type
{
public:
  predicate(core &cr, scope &scp, const std::string &name, const std::vector<field *> &args);
  predicate(const predicate &orig) = delete;
  virtual ~predicate();

  const std::vector<field *> get_args() const { return args; }

protected:
  const std::vector<field *> args;
};
}