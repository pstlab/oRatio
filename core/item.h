#pragma once

#include "env.h"
#include "var_value.h"

namespace ratio
{

class type;

class item : public env, public smt::var_value
{
public:
  item(core &cr, const type &tp);
  item(const item &orig) = delete;
  virtual ~item();

  const type &get_type() { return tp; }

private:
  const type &tp;
};
} // namespace ratio
