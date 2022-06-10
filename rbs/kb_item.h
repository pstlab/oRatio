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

    virtual bool eq(const item &it) const noexcept = 0;
  };

  class bool_item : public item
  {
  public:
    bool_item(bool val);
    ~bool_item() = default;

    bool eq(const item &it) const noexcept override;

    inline bool get() const noexcept { return val; }

  private:
    bool val;
  };

  class arith_item : public item
  {
  public:
    arith_item(smt::rational val);
    ~arith_item() = default;

    bool eq(const item &it) const noexcept override;

    inline smt::rational get() const noexcept { return val; }

  private:
    smt::rational val;
  };

  class string_item : public item
  {
  public:
    string_item(const std::string &val);
    ~string_item() = default;

    bool eq(const item &it) const noexcept override;

    inline std::string get() const noexcept { return val; }

  private:
    std::string val;
  };

  using expr = std::shared_ptr<item>;
} // namespace kb
