#pragma once

#include "rbs_export.h"
#include "kb_item.h"
#include <vector>
#include <unordered_map>

namespace rbs::ast
{
  class compilation_unit;
} // namespace rbs::ast

namespace kb
{
  class predicate;
  class fact;
  class rule;

  class knowledge_base
  {
  public:
    RBS_EXPORT knowledge_base();
    RBS_EXPORT ~knowledge_base();

    RBS_EXPORT void read(const std::string &script);             // parses the given rbs script..
    RBS_EXPORT void read(const std::vector<std::string> &files); // parses the given rbs files..

    RBS_EXPORT expr get(const std::string &fn) const { return xprs.at(fn); }
    RBS_EXPORT bool exists_predicate(const std::string &p_name) const noexcept { return predicates.count(p_name); }

  private:
    void set(const std::string &fn, expr xpr);
    predicate &create_predicate(const std::string &p_name);

  private:
    std::vector<rbs::ast::compilation_unit *> cus; // the compilation units..

    std::unordered_map<std::string, predicate *> predicates;
    std::unordered_map<std::string, const fact *> facts;
    std::unordered_map<std::string, const rule *> rules;

    std::unordered_map<std::string, expr> xprs;
  };
} // namespace kb
