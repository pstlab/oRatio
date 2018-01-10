#pragma once

#include "scope.h"

namespace ratio
{

class conjunction;

class disjunction : public scope
{
public:
  disjunction(core &cr, scope &scp, const std::vector<const conjunction *> &conjs);
  disjunction(const disjunction &orig) = delete;
  virtual ~disjunction();

  const std::vector<const conjunction *> get_conjunctions() const { return conjunctions; }

private:
  const std::vector<const conjunction *> conjunctions;
};
}