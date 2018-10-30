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
  friend class core;
  friend class var_item;

public:
  env(core &cr, const context ctx);
  env(const env &orig) = delete;
  ~env();

  core &get_core() const { return cr; }   // returns the core in which this environment is created..
  context get_ctx() const { return ctx; } // returns the context in which this environment is created..

  virtual expr get(const std::string &name) const;                         // returns the expression having the given name, checks in the enclosing environment if the name is not found..
  std::map<std::string, expr> get_exprs() const noexcept { return exprs; } // returns a map of names and their corresponding expressions directly accessible from this environment..

private:
  core &cr; // the core in which this environment is created..

private:
  unsigned ref_count; // the number of references for this environment (used for implementing a 'smart pointer' infrastructure)..

private:
  const context ctx;                 // the context in which this environment was created..
  std::map<std::string, expr> exprs; // the expressions defined within this environment..
};
} // namespace ratio
