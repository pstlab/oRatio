#pragma once

#include <map>
#include <string>
#include <vector>

namespace ratio
{

class core;

class scope
{
public:
  scope(core &cr, scope &scp);
  scope(const scope &orig) = delete;
  ~scope();

private:
  core &cr;   // the core in which this scope is defined..
  scope &scp; // the enclosing scope..
};
} // namespace ratio
