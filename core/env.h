#pragma once

#include "context.h"
#include <map>
#include <string>

namespace ratio
{

class core;
class context;

class env
{
  friend class context;

public:
  env(core &cr, const context ctx);
  env(const env &orig) = delete;
  ~env();

  core &get_core() const { return cr; }   // returns the core in which this environment is created..
  context get_ctx() const { return ctx; } // returns the context in which this environment is created..

private:
  core &cr; // the core in which this environment is created..

private:
  unsigned ref_count = 0; // the number of references for this environment (used for implementing a smart pointer infrastructure)..

private:
  const context ctx; // the context in which this environment was created..
};
} // namespace ratio
