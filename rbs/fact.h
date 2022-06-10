#pragma once

#include "kb_item.h"

namespace kb
{
  class node;
  class fact;

  class predicate
  {
    friend class fact;

  public:
    predicate(const std::string &name);
    ~predicate();

    fact &new_instance() noexcept;

  private:
    const std::string name;               // the name of this type..
    std::unordered_set<fact *> instances; // a set containing all the instances of this type..
  };

  class fact : public item
  {
  public:
    fact(predicate &p);
    ~fact();

    predicate &get_predicate() const noexcept { return pred; }
    expr get(const std::string &fn) const { return xprs.at(fn); }
    void set(const std::string &fn, expr xpr);

  private:
    predicate &pred;
    std::map<std::string, expr> xprs;
    std::vector<const fact *> support_facts;    // the facts supported by this fact.. if this fact is removed, also the facts supported by this fact must be removed..
    std::unordered_set<node *> evaluated_nodes; // the nodes in which this fact has been positively evaluated..
  };
} // namespace kb
