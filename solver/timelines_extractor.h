#pragma once

#include "solver_export.h"
#include "json.h"

namespace ratio
{
  class timelines_extractor
  {
  public:
    SOLVER_EXPORT timelines_extractor() {}
    SOLVER_EXPORT virtual ~timelines_extractor() {}

    virtual smt::json extract_timelines() const noexcept = 0;
  };
} // namespace ratio
