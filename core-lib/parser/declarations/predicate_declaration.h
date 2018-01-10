#pragma once

#include <vector>
#include <string>

namespace ratio
{

class scope;

namespace ast
{

class statement;

class predicate_declaration
{
public:
  predicate_declaration(const std::string &n, const std::vector<std::pair<std::vector<std::string>, std::string>> &pars, const std::vector<std::vector<std::string>> &pl, const std::vector<statement *> &stmnts);
  predicate_declaration(const predicate_declaration &orig) = delete;
  virtual ~predicate_declaration();

  void refine(scope &scp) const;

private:
  const std::string name;
  const std::vector<std::pair<std::vector<std::string>, std::string>> parameters;
  const std::vector<std::vector<std::string>> predicate_list;
  const std::vector<statement *> statements;
};
}
}