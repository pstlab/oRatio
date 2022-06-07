#pragma once

#include <unordered_map>

namespace kb
{
  class predicate;
  class fact;
  class rule;

  class knowledge_base
  {
  public:
    knowledge_base();
    ~knowledge_base();

    bool exists_predicate(const std::string &p_name) { return predicates.count(p_name); }

  private:
    std::unordered_map<std::string, predicate *> predicates;
    std::unordered_map<std::string, const fact *> facts;
    std::unordered_map<std::string, const rule *> rules;
  };
} // namespace kb
