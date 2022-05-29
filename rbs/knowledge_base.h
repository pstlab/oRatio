#pragma once

#include <unordered_map>

namespace rbs
{
  class predicate;
  class fact;
  class rule;

  class knowledge_base
  {
  public:
    knowledge_base();
    ~knowledge_base();

  private:
    std::unordered_map<std::string, predicate *> predicates;
    std::unordered_map<std::string, const fact *> facts;
    std::unordered_map<std::string, const rule *> rules;
  };
} // namespace rbs
