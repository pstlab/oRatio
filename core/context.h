#pragma once

#include "core_export.h"

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
   * @brief This class is intended to implement a kinf of smart pointer infrastructure.
   * This simple implementation allows an efficient management of memory for the expressions.
   * 
   * Roughly speaking, a context is a reference to an environment. It can be used to manage the environment's lifecycle.
   * In case a method is invoked, for example, a new environment is created for the whole method invocation and deleted just after the method returns.
   */
  class context
  {
  public:
    CORE_EXPORT explicit context(env *const ptr);
    CORE_EXPORT explicit context(const context &orig);
    CORE_EXPORT virtual ~context();

    inline env &operator*() const { return *ptr; }
    inline env *operator->() const { return ptr; }

    operator expr() const;
    CORE_EXPORT operator bool_expr() const;
    CORE_EXPORT operator arith_expr() const;
    CORE_EXPORT operator var_expr() const;
    CORE_EXPORT operator string_expr() const;

    inline bool operator==(const context &right) const noexcept { return ptr == right.ptr; }
    inline bool operator!=(const context &right) const noexcept { return !(*this == right); }

  protected:
    env *const ptr;
  };

  class expr : public context
  {
  public:
    expr(item *const ptr);
    expr(const expr &orig) : context(orig.ptr) {}
    virtual ~expr() {}

    CORE_EXPORT item &operator*() const;
    CORE_EXPORT item *operator->() const;
  };

  class bool_expr : public expr
  {
  public:
    CORE_EXPORT bool_expr(bool_item *const ptr);
    CORE_EXPORT bool_expr(const bool_expr &orig);
    CORE_EXPORT virtual ~bool_expr() {}

    CORE_EXPORT bool_item &operator*() const;
    CORE_EXPORT bool_item *operator->() const;
  };

  class arith_expr : public expr
  {
  public:
    CORE_EXPORT arith_expr(arith_item *const ptr);
    CORE_EXPORT arith_expr(const arith_expr &orig);
    CORE_EXPORT virtual ~arith_expr() {}

    CORE_EXPORT arith_item &operator*() const;
    CORE_EXPORT arith_item *operator->() const;
  };

  class var_expr : public expr
  {
  public:
    CORE_EXPORT var_expr(var_item *const ptr);
    CORE_EXPORT var_expr(const var_expr &orig);
    CORE_EXPORT virtual ~var_expr() {}

    CORE_EXPORT var_item &operator*() const;
    CORE_EXPORT var_item *operator->() const;
  };

  class string_expr : public expr
  {
  public:
    CORE_EXPORT string_expr(string_item *const ptr);
    CORE_EXPORT string_expr(const string_expr &orig);
    CORE_EXPORT virtual ~string_expr() {}

    CORE_EXPORT string_item &operator*() const;
    CORE_EXPORT string_item *operator->() const;
  };
} // namespace ratio