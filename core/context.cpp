#include "context.h"
#include "item.h"

namespace ratio
{
    CORE_EXPORT context::context(env *const ptr) : ptr(ptr) { ptr->ref_count++; }
    CORE_EXPORT context::context(const context &orig) : ptr(orig.ptr) { ptr->ref_count++; }
    CORE_EXPORT context::~context()
    {
        ptr->ref_count--;
        if (ptr->ref_count == 0) // there are no more references to this environment..
            delete ptr;          // we can hence delete the environment..
    }

    context::operator expr() const { return expr(static_cast<item *>(ptr)); }
    CORE_EXPORT context::operator bool_expr() const { return bool_expr(static_cast<bool_item *>(ptr)); }
    CORE_EXPORT context::operator arith_expr() const { return arith_expr(static_cast<arith_item *>(ptr)); }
    CORE_EXPORT context::operator string_expr() const { return string_expr(static_cast<string_item *>(ptr)); }
    CORE_EXPORT context::operator var_expr() const { return var_expr(static_cast<var_item *>(ptr)); }

    expr::expr(item *const ptr) : context(ptr) {}
    CORE_EXPORT item &expr::operator*() const { return *static_cast<item *>(ptr); }
    CORE_EXPORT item *expr::operator->() const { return static_cast<item *>(ptr); }

    CORE_EXPORT bool_expr::bool_expr(bool_item *const ptr) : expr(ptr) {}
    CORE_EXPORT bool_expr::bool_expr(const bool_expr &orig) : expr(static_cast<item *>(orig.ptr)) {}
    CORE_EXPORT bool_item &bool_expr::operator*() const { return *static_cast<bool_item *>(ptr); }
    CORE_EXPORT bool_item *bool_expr::operator->() const { return static_cast<bool_item *>(ptr); }

    CORE_EXPORT arith_expr::arith_expr(arith_item *const ptr) : expr(ptr) {}
    CORE_EXPORT arith_expr::arith_expr(const arith_expr &orig) : expr(static_cast<item *>(orig.ptr)) {}
    CORE_EXPORT arith_item &arith_expr::operator*() const { return *static_cast<arith_item *>(ptr); }
    CORE_EXPORT arith_item *arith_expr::operator->() const { return static_cast<arith_item *>(ptr); }

    CORE_EXPORT var_expr::var_expr(var_item *const ptr) : expr(ptr) {}
    CORE_EXPORT var_expr::var_expr(const var_expr &orig) : expr(static_cast<item *>(orig.ptr)) {}
    CORE_EXPORT var_item &var_expr::operator*() const { return *static_cast<var_item *>(ptr); }
    CORE_EXPORT var_item *var_expr::operator->() const { return static_cast<var_item *>(ptr); }

    CORE_EXPORT string_expr::string_expr(string_item *const ptr) : expr(ptr) {}
    CORE_EXPORT string_expr::string_expr(const string_expr &orig) : expr(static_cast<item *>(orig.ptr)) {}
    CORE_EXPORT string_item &string_expr::operator*() const { return *static_cast<string_item *>(ptr); }
    CORE_EXPORT string_item *string_expr::operator->() const { return static_cast<string_item *>(ptr); }
} // namespace ratio