#pragma once

#include "scope.h"
#include "env.h"
#include "sat_core.h"
#include "lra_theory.h"
#include "ov_theory.h"

namespace ratio
{

class core : public scope, public env
{
public:
  core();
  core(const core &orig) = delete;
  ~core();

public:
  smt::sat_core sat_cr;   // the sat core..
  smt::lra_theory lra_th; // the linear-real-arithmetic theory..
  smt::ov_theory ov_th;   // the object-variable theory..
};
} // namespace ratio