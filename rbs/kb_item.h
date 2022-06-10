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
  };

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

  using expr = std::shared_ptr<item>;
} // namespace kb
