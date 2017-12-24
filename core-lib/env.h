#pragma once

#include "context.h"
#include <map>
#include <string>

namespace ratio
{

class core;
class constructor;
class method;
class predicate;
class var_item;

class env
{

  friend class core;
  friend class context;
  friend class constructor;
  friend class method;
  friend class predicate;
  friend class var_item;

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