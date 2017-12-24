#pragma once

#include "context.h"
#include <map>
#include <string>

namespace ratio
{

class core;

class env
{

  friend class context;

public:
  env(core &cr, const context ctx);
  env(const env &orig) = delete;
  virtual ~env();

  core &get_core() const { return cr; }
  context get_ctx() const { return ctx; }

  virtual expr get(const std::string &name) const;
  std::map<std::string, expr> get_items() const noexcept { return items; }

private:
  unsigned ref_count;

protected:
  core &cr;

private:
  const context ctx;
  std::map<std::string, expr> items;
};
}