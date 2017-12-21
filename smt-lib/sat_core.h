#pragma once

#include "lit.h"
#include "lbool.h"

namespace smt
{
static const var FALSE_var = 0;
static const var TRUE_var = 1;

class sat_core
{
public:
  sat_core();
  sat_core(const sat_core &orig) = delete;
  virtual ~sat_core();
};
}