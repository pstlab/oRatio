#pragma once

namespace ratio
{

  class env;
  class item;
  class bool_item;
  class arith_item;
  class var_item;
  class string_item;

  class expr;
  class bool_expr;
  class arith_expr;
  class var_expr;
  class string_expr;

  /**
   * This class is intended to implement a kinf of smart pointer infrastructure.
   * This simple implementation allows an efficient management of memory for the expressions.
   * 
   * Roughly speaking, a context is a reference to an environment. It can be used to manage the environment's lifecycle.
   * In case a method is invoked, for example, a new environment is created for the whole method invocation and deleted just after the method returns.
   */
  class context
  {
  public:
    context(env *const ptr);
    context(const context &orig);
    virtual ~context();

    env &operator*() const { return *ptr; }
    env *operator->() const { return ptr; }

    operator expr() const;
    operator bool_expr() const;
    operator arith_expr() const;
    operator var_expr() const;
    operator string_expr() const;

    bool operator==(const context &right) const noexcept { return ptr == right.ptr; }
    bool operator!=(const context &right) const noexcept { return !(*this == right); }

  protected:
    env *const ptr;
  };

  class expr : public context
  {
  public:
    expr(item *const ptr);
    expr(const expr &orig) : context(orig.ptr) {}
    virtual ~expr() {}

    item &operator*() const;
    item *operator->() const;
  };

  class bool_expr : public expr
  {
  public:
    bool_expr(bool_item *const ptr);
    bool_expr(const bool_expr &orig);
    virtual ~bool_expr() {}

    bool_item &operator*() const;
    bool_item *operator->() const;
  };

  class arith_expr : public expr
  {
  public:
    arith_expr(arith_item *const ptr);
    arith_expr(const arith_expr &orig);
    virtual ~arith_expr() {}

    arith_item &operator*() const;
    arith_item *operator->() const;
  };

  class var_expr : public expr
  {
  public:
    var_expr(var_item *const ptr);
    var_expr(const var_expr &orig);
    virtual ~var_expr() {}

    var_item &operator*() const;
    var_item *operator->() const;
  };

  class string_expr : public expr
  {
  public:
    string_expr(string_item *const ptr);
    string_expr(const string_expr &orig);
    virtual ~string_expr() {}

    string_item &operator*() const;
    string_item *operator->() const;
  };
} // namespace ratio