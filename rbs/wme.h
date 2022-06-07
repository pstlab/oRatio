#pragma once

#include "rational.h"
#include <memory>
#include <vector>
#include <map>

namespace kb
{
  class predicate;

  class item
  {
  public:
    item() = default;
    virtual ~item() = default;
  };

  using expr = std::shared_ptr<item>;

  class bool_item : public item
  {
  public:
    bool_item(bool val) : val(val) {}
    ~bool_item() = default;

    bool val;
  };

  class arith_item : public item
  {
  public:
    arith_item(smt::rational val) : val(val) {}
    ~arith_item() = default;

    smt::rational val;
  };

  class string_item : public item
  {
  public:
    string_item(const std::string &val) : val(val) {}
    ~string_item() = default;

    std::string val;
  };

  class fact
  {
  public:
    fact(const predicate &p) : pred(p) {}
    ~fact() = default;

    const predicate &get_predicate() const noexcept { return pred; }
    expr get(const std::string &fn) const { return xprs.at(fn); }

  private:
    const predicate &pred;
    std::map<std::string, expr> xprs;
    std::vector<const fact *> supports;
  };
} // namespace kb
