#pragma once

#include "scope.h"

namespace ratio
{

class expr;
class context;
class predicate;

class type : public scope
{
  friend class predicate;

public:
  type(core &cr, scope &scp, const std::string &name, bool primitive = false);
  type(const type &orig) = delete;
  virtual ~type();

  std::string get_name() const { return name; }                              // returns the name of this type..
  bool is_primitive() const { return primitive; }                            // returns whether this type is primitive..
  std::vector<type *> get_supertypes() const noexcept { return supertypes; } // returns the base types of this type..

  bool is_assignable_from(const type &t) const noexcept; // checks whether this type is assignable from the 't' type..

  virtual expr new_instance(context &ctx); // creates a new instance of this type within the given context..
  virtual expr new_existential();          // creates a new existential of this type (i.e. an object variable whose allowed values are all the current instances of this type)..

  field &get_field(const std::string &name) const override; // returns the field having the given name..

protected:
  static void new_supertypes(type &t, const std::vector<type *> &sts)
  {
    for (const auto &st : sts)
      t.supertypes.push_back(st);
  }

private:
  const std::string name;         // the name of this type..
  const bool primitive;           // is this type a primitive type?
  std::vector<type *> supertypes; // the base types (i.e. the types this type inherits from)..
  std::vector<expr> instances;    // a vector containing all the instances of this type..
};

class bool_type : public type
{
public:
  bool_type(core &cr);
  bool_type(const bool_type &that) = delete;
  virtual ~bool_type();
};

class int_type : public type
{
public:
  int_type(core &cr);
  int_type(const int_type &that) = delete;
  virtual ~int_type();
};

class real_type : public type
{
public:
  real_type(core &cr);
  real_type(const real_type &that) = delete;
  virtual ~real_type();
};

class string_type : public type
{
public:
  string_type(core &cr);
  string_type(const string_type &that) = delete;
  virtual ~string_type();
};
} // namespace ratio
