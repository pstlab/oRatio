#pragma once

#include <vector>
#include <string>

namespace lucy
{

class scope;

namespace ast
{

class expression;
class statement;

class constructor_declaration
{
public:
  constructor_declaration(const std::vector<std::pair<std::vector<std::string>, std::string>> &pars, const std::vector<std::pair<std::string, std::vector<expression *>>> &il, const std::vector<statement *> &stmnts);
  constructor_declaration(const constructor_declaration &orig) = delete;
  virtual ~constructor_declaration();

  void refine(scope &scp) const;

private:
  const std::vector<std::pair<std::vector<std::string>, std::string>> parameters;
  const std::vector<std::pair<std::string, std::vector<expression *>>> init_list;
  const std::vector<statement *> statements;
};
}
}