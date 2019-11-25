#pragma once

#include "env.h"
#include "lit.h"
#include "lin.h"
#include "var_value.h"

namespace ratio
{

class type;

class item : public env, public smt::var_value
{
public:
  item(core &cr, const context ctx, const type &tp);
  item(const item &orig) = delete;
  virtual ~item();

  const type &get_type() const { return tp; }

  virtual smt::var eq(item &i) noexcept;
  virtual bool equates(const item &i) const noexcept;

private:
  const type &tp;
};

class bool_item : public item
{
public:
  bool_item(core &cr, const smt::lit &l);
  bool_item(const bool_item &that) = delete;
  virtual ~bool_item();

  smt::var eq(item &i) noexcept override;
  bool equates(const item &i) const noexcept override;

public:
  smt::lit l;
};

class arith_item : public item
{
public:
  arith_item(core &cr, const type &t, const smt::lin &l);
  arith_item(const arith_item &that) = delete;
  virtual ~arith_item();

  smt::var eq(item &i) noexcept override;
  bool equates(const item &i) const noexcept override;

public:
  const smt::lin l;
};

class var_item : public item
{
public:
  var_item(core &cr, const type &t, smt::var ev);
  var_item(const var_item &that) = delete;
  virtual ~var_item();

  expr get(const std::string &name) const override;

  smt::var eq(item &i) noexcept override;
  bool equates(const item &i) const noexcept override;

public:
  const smt::var ev;
};

class string_item : public item
{
public:
  string_item(core &cr, const std::string &l);
  string_item(const string_item &that) = delete;
  virtual ~string_item();

  std::string get_value() { return l; }

  smt::var eq(item &i) noexcept override;
  bool equates(const item &i) const noexcept override;

private:
  std::string l;
};
} // namespace ratio
