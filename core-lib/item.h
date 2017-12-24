#pragma once

#include "env.h"
#include "lit.h"
#include "lin.h"
#include "var_value.h"

using namespace smt;

namespace ratio
{

class type;

class item : public env, public var_value
{
  public:
    item(core &cr, const context ctx, const type &tp);
    item(const item &orig) = delete;
    virtual ~item();

    virtual var eq(item &i) noexcept;
    virtual bool equates(const item &i) const noexcept;

  public:
    const type &tp;
};

class bool_item : public item
{
  public:
    bool_item(core &cr, const lit &l);
    bool_item(const bool_item &that) = delete;
    virtual ~bool_item();

    var eq(item &i) noexcept override;
    bool equates(const item &i) const noexcept override;

  public:
    lit l;
};

class arith_item : public item
{
  public:
    arith_item(core &cr, const type &t, const lin &l);
    arith_item(const arith_item &that) = delete;
    virtual ~arith_item();

    var eq(item &i) noexcept override;
    bool equates(const item &i) const noexcept override;

  public:
    const lin l;
};

class string_item : public item
{
  public:
    string_item(core &cr, const std::string &l);
    string_item(const string_item &that) = delete;
    virtual ~string_item();

    std::string get_value() { return l; }

    var eq(item &i) noexcept override;
    bool equates(const item &i) const noexcept override;

  private:
    std::string l;
};

class var_item : public item
{
  public:
    var_item(core &cr, const type &t, var ev);
    var_item(const var_item &that) = delete;
    virtual ~var_item();

    expr get(const std::string &name) const override;

    var eq(item &i) noexcept override;
    bool equates(const item &i) const noexcept override;

  public:
    const var ev;
};
}