#pragma once

#include <map>
#include <string>
#include <vector>

#define THIS_KEYWORD "this"

namespace ratio
{

class core;
class field;
class type;
class predicate;

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

  virtual field &get_field(const std::string &name) const;    // returns the field having the given name, check in the enclosed scope if the field is not found..
  std::map<std::string, field *> get_fields() const noexcept; // returns a map of fields defined within this scope having the fields' names as keys..

  virtual type &get_type(const std::string &name) const;            // returns the type having the given name, check in the enclosed scope if the type is not found..
  virtual std::map<std::string, type *> get_types() const noexcept; // returns a map of types defined within this scope having the types' names as keys..

  virtual predicate &get_predicate(const std::string &name) const;            // returns the predicate having the given name, check in the enclosed scope if the predicate is not found..
  virtual std::map<std::string, predicate *> get_predicates() const noexcept; // returns a map of predicates defined within this scope having the predicates' names as keys..

protected:
  static void new_fields(scope &s, const std::vector<field *> &fs);
  void new_fields(const std::vector<field *> &fs);

private:
  core &cr;   // the core in which this scope is defined..
  scope &scp; // the enclosing scope..

private:
  std::map<std::string, field *> fields; // the fields of this scope..
};
} // namespace ratio
