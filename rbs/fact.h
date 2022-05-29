#pragma once

#include <vector>

namespace rbs
{
  class predicate;

  class fact
  {
  public:
    fact(const predicate &p);
    ~fact();

  private:
    const predicate &pred;
    std::vector<const fact *> supports;
  };
} // namespace rbs
