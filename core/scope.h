#pragma once

#include <map>
#include <string>
#include <vector>

#define THIS_KEYWORD "this"
#define RETURN_KEYWORD "return"
#define TAU "tau"

namespace ratio
{

class core;
class field;
class method;
class type;
class predicate;

namespace ast
{
class local_field_statement;
class field_declaration;
} // namespace ast

class scope
{
  friend class core;
  friend class type;
  friend class ast::local_field_statement;
  friend class ast::field_declaration;

public:
  scope(core &cr, scope &scp);
  scope(const scope &orig) = delete;
  ~scope();

  core &get_core() const { return cr; }    // returns the core in which this scope is defined..
  scope &get_scope() const { return scp; } // returns the enclosing scope..

  virtual const field &get_field(const std::string &name) const;    // returns the field having the given name, check in the enclosed scope if the field is not found..
  std::map<std::string, const field *> get_fields() const noexcept; // returns a map of fields defined within this scope having the fields' names as keys..

  virtual const method &get_method(const std::string &name, const std::vector<const type *> &ts) const; // returns the method having the given name and the given argument types, check in the enclosed scope if the type is not found..
  virtual std::vector<const method *> get_methods() const noexcept;                                     // returns the vector of methods defined within this scope..

  virtual type &get_type(const std::string &name) const;            // returns the type having the given name, check in the enclosed scope if the type is not found..
  virtual std::map<std::string, type *> get_types() const noexcept; // returns a map of types defined within this scope having the types' names as keys..

  virtual predicate &get_predicate(const std::string &name) const;            // returns the predicate having the given name, check in the enclosed scope if the predicate is not found..
  virtual std::map<std::string, predicate *> get_predicates() const noexcept; // returns a map of predicates defined within this scope having the predicates' names as keys..

protected:
  static void new_fields(scope &s, const std::vector<const field *> &fs);
  void new_fields(const std::vector<const field *> &fs);

private:
  core &cr;   // the core in which this scope is defined..
  scope &scp; // the enclosing scope..

private:
  std::map<std::string, const field *> fields; // the fields of this scope..
};
} // namespace ratio
