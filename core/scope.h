#pragma once

#include <map>
#include <string>
#include <vector>

namespace ratio
{

class core;
class field;
class type;

class scope
{
  friend class core;
  friend class type;

public:
  scope(core &cr, scope &scp);
  scope(const scope &orig) = delete;
  ~scope();

  core &get_core() const { return cr; }    // returns the core in which this scope is defined..
  scope &get_scope() const { return scp; } // returns the enclosing scope..

  virtual field &get_field(const std::string &name) const;    // returns the field having the given name..
  std::map<std::string, field *> get_fields() const noexcept; // returns a map of fields having the fields' name as keys..

private:
  core &cr;   // the core in which this scope is defined..
  scope &scp; // the enclosing scope..

private:
  std::map<std::string, field *> fields; // the fields of this scope..
};
} // namespace ratio
