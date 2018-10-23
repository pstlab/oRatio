#pragma once

#include "context.h"
#include <map>
#include <string>

namespace ratio
{

class core;

class env
{
public:
  env(core &cr);
  env(const env &orig) = delete;
  ~env();

private:
  core &cr;
};
} // namespace ratio
