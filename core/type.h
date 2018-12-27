#pragma once

#include "scope.h"

namespace ratio
{

class expr;
class context;
class constructor;
class enum_type;

namespace ast
{
class method_declaration;
class predicate_declaration;
class typedef_declaration;
class enum_declaration;
class constructor_declaration;
class class_declaration;
} // namespace ast

class type : public scope
{
  friend class predicate;
  friend class enum_type;
  friend class ast::method_declaration;
  friend class ast::predicate_declaration;
  friend class ast::typedef_declaration;
  friend class ast::enum_declaration;
  friend class ast::constructor_declaration;
  friend class ast::class_declaration;

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

  std::vector<expr> get_instances() const noexcept { return instances; } // returns the instances of this type..

protected:
  static void new_supertypes(type &t, const std::vector<type *> &sts);
  void new_supertypes(const std::vector<type *> &sts);
  void new_constructors(const std::vector<const constructor *> &cs);
  void new_methods(const std::vector<const method *> &ms);
  void new_types(const std::vector<type *> &ts);
  void new_predicates(const std::vector<predicate *> &ps);

public:
  const constructor &get_constructor(const std::vector<const type *> &ts) const;
  std::vector<const constructor *> get_constructors() const noexcept { return constructors; }

  const field &get_field(const std::string &name) const override; // returns the field having the given name..

  const method &get_method(const std::string &m_name, const std::vector<const type *> &ts) const override;
  std::vector<const method *> get_methods() const noexcept override
  {
    std::vector<const method *> c_methods;
    for (const auto &ms : methods)
      c_methods.insert(c_methods.begin(), ms.second.begin(), ms.second.end());
    return c_methods;
  }

  type &get_type(const std::string &t_name) const override;
  std::map<std::string, type *> get_types() const noexcept override { return types; }

  predicate &get_predicate(const std::string &p_name) const override;
  std::map<std::string, predicate *> get_predicates() const noexcept override { return predicates; }

private:
  virtual void new_predicate(predicate &) {}

private:
  const std::string name;                                     // the name of this type..
  const bool primitive;                                       // is this type a primitive type?
  std::vector<type *> supertypes;                             // the base types (i.e. the types this type inherits from)..
  std::vector<const constructor *> constructors;              // the constructors defined within this type..
  std::map<std::string, std::vector<const method *>> methods; // the methods, indexed by their name, defined within this type..
  std::map<std::string, type *> types;                        // the inner types, indexed by their name, defined within this type..
  std::map<std::string, predicate *> predicates;              // the inner predicates, indexed by their name, defined within this type..
  std::vector<expr> instances;                                // a vector containing all the instances defined within this type..
};

class bool_type : public type
{
public:
  bool_type(core &cr);
  bool_type(const bool_type &that) = delete;
  virtual ~bool_type();

  expr new_instance(context &ctx) override;
};

class int_type : public type
{
public:
  int_type(core &cr);
  int_type(const int_type &that) = delete;
  virtual ~int_type();

  expr new_instance(context &ctx) override;
};

class real_type : public type
{
public:
  real_type(core &cr);
  real_type(const real_type &that) = delete;
  virtual ~real_type();

  expr new_instance(context &ctx) override;
};

class string_type : public type
{
public:
  string_type(core &cr);
  string_type(const string_type &that) = delete;
  virtual ~string_type();

  expr new_instance(context &ctx) override;
};
} // namespace ratio
