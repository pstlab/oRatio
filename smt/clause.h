#pragma once

#include "lit.h"
#include <vector>

namespace smt
{

class sat_core;

/**
 * This class is used for representing propositional constraints.
 */
class clause
{
  friend class sat_core;

private:
  clause(sat_core &s, const std::vector<lit> &lits);
  clause(const clause &orig) = delete;
  ~clause();

public:
  const std::vector<lit> get_lits() const { return lits; }

private:
  const bool propagate(const lit &p);

private:
  sat_core &s;
  std::vector<lit> lits;
};
} // namespace smt
