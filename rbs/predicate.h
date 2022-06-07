#pragma once

#include <string>
#include <vector>

namespace kb
{
  class fact;
  class rule;

  class predicate
  {
  public:
    predicate(const std::string &name);
    ~predicate();

  private:
    const std::string name;        // the name of this type..
    std::vector<fact *> instances; // a vector containing all the instances of this type..
    std::vector<rule *> rules;     // a vector containing all the rules in which this type appears..
  };
} // namespace kb
