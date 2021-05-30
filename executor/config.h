#pragma once

#include "executor_export.h"
#include <string>
#include <istream>
#include <unordered_set>

namespace ratio
{
  class solver;
  class predicate;

  class config
  {
  public:
    EXECUTOR_EXPORT config();
    config(const config &orig) = delete;
    EXECUTOR_EXPORT ~config();

    EXECUTOR_EXPORT void read(const solver &slv, const std::string &cnfg);
    EXECUTOR_EXPORT void read(const solver &slv, std::istream &is);

    inline const std::unordered_set<predicate *> &get_notify_start() const noexcept { return notify_start; }
    inline const std::unordered_set<predicate *> &get_notify_end() const noexcept { return notify_end; }
    inline const std::unordered_set<predicate *> &get_auto_start() const noexcept { return auto_start; }
    inline const std::unordered_set<predicate *> &get_auto_end() const noexcept { return auto_end; }

  private:
    std::unordered_set<predicate *> notify_start;
    std::unordered_set<predicate *> notify_end;
    std::unordered_set<predicate *> auto_start;
    std::unordered_set<predicate *> auto_end;
  };
} // namespace ratio
