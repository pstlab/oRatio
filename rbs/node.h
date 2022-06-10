#pragma once

#include <string>
#include <vector>
#include <unordered_set>

namespace kb
{
  class rule;
  class item;
  class predicate;

  class node
  {
  public:
    node() = default;
    virtual ~node() = default;

  private:
    std::unordered_set<item *> evaluated_items;
    std::unordered_set<node *> previous_nodes, next_nodes;
    std::vector<rule *> rules; // a vector containing all the rules in which this node appears..
  };

  class predicate_node : public node
  {
  public:
    predicate_node(const std::string &id, const predicate &pred);
    ~predicate_node();

  private:
    const std::string id;
    const predicate &pred; // the predicate represented by this node..
  };

  class condition_node : public node
  {
  public:
    condition_node();
    ~condition_node();
  };
} // namespace kb
