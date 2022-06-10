#pragma once

#include "rational.h"
#include <memory>
#include <vector>
#include <map>
#include <unordered_set>

namespace kb
{
  class item
  {
  public:
    item() = default;
    virtual ~item() = default;

    virtual bool eq(const item &it) = 0;
  };

  class bool_item : public item
  {
  public:
    bool_item(bool val) : val(val) {}
    ~bool_item() = default;

    bool eq(const item &it)
    {
      if (const bool_item *bi = dynamic_cast<const bool_item *>(&it))
        return val == bi->val;
      else
        return false;
    }

    inline bool get() const noexcept { return val; }

  private:
    bool val;
  };

  class arith_item : public item
  {
  public:
    arith_item(smt::rational val) : val(val) {}
    ~arith_item() = default;

    bool eq(const item &it)
    {
      if (const arith_item *ai = dynamic_cast<const arith_item *>(&it))
        return val == ai->val;
      else
        return false;
    }

    inline smt::rational get() const noexcept { return val; }

  private:
    smt::rational val;
  };

  class string_item : public item
  {
  public:
    string_item(const std::string &val) : val(val) {}
    ~string_item() = default;

    bool eq(const item &it)
    {
      if (const string_item *si = dynamic_cast<const string_item *>(&it))
        return val == si->val;
      else
        return false;
    }

    inline std::string get() const noexcept { return val; }

  private:
    std::string val;
  };

  using expr = std::shared_ptr<item>;
} // namespace kb
