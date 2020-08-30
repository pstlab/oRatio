#include "context.h"
#include "item.h"

namespace ratio
{

    context::context(env *const ptr) : ptr(ptr) { ptr->ref_count++; }
    context::context(const context &orig) : ptr(orig.ptr) { ptr->ref_count++; }
    context::~context()
    {
        ptr->ref_count--;
        if (ptr->ref_count == 0) // there are no more references to this environment..
            delete ptr;          // we can hence delete the environment..
    }

    context::operator expr() const { return expr(static_cast<item *>(ptr)); }
    context::operator bool_expr() const { return bool_expr(static_cast<bool_item *>(ptr)); }
    context::operator arith_expr() const { return arith_expr(static_cast<arith_item *>(ptr)); }
    context::operator string_expr() const { return string_expr(static_cast<string_item *>(ptr)); }
    context::operator var_expr() const { return var_expr(static_cast<var_item *>(ptr)); }

    expr::expr(item *const ptr) : context(ptr) {}
    item &expr::operator*() const { return *static_cast<item *>(ptr); }
    item *expr::operator->() const { return static_cast<item *>(ptr); }

    bool_expr::bool_expr(bool_item *const ptr) : expr(ptr) {}
    bool_expr::bool_expr(const bool_expr &orig) : expr(static_cast<item *>(orig.ptr)) {}
    bool_item &bool_expr::operator*() const { return *static_cast<bool_item *>(ptr); }
    bool_item *bool_expr::operator->() const { return static_cast<bool_item *>(ptr); }

    arith_expr::arith_expr(arith_item *const ptr) : expr(ptr) {}
    arith_expr::arith_expr(const arith_expr &orig) : expr(static_cast<item *>(orig.ptr)) {}
    arith_item &arith_expr::operator*() const { return *static_cast<arith_item *>(ptr); }
    arith_item *arith_expr::operator->() const { return static_cast<arith_item *>(ptr); }

    var_expr::var_expr(var_item *const ptr) : expr(ptr) {}
    var_expr::var_expr(const var_expr &orig) : expr(static_cast<item *>(orig.ptr)) {}
    var_item &var_expr::operator*() const { return *static_cast<var_item *>(ptr); }
    var_item *var_expr::operator->() const { return static_cast<var_item *>(ptr); }

    string_expr::string_expr(string_item *const ptr) : expr(ptr) {}
    string_expr::string_expr(const string_expr &orig) : expr(static_cast<item *>(orig.ptr)) {}
    string_item &string_expr::operator*() const { return *static_cast<string_item *>(ptr); }
    string_item *string_expr::operator->() const { return static_cast<string_item *>(ptr); }
} // namespace ratio