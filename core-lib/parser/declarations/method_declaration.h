#pragma once

#include <vector>
#include <string>

namespace ratio
{

class scope;

namespace ast
{

class statement;

class method_declaration
{
public:
  method_declaration(const std::vector<std::string> &rt, const std::string &n, const std::vector<std::pair<std::vector<std::string>, std::string>> &pars, const std::vector<statement *> &stmnts);
  method_declaration(const method_declaration &orig) = delete;
  virtual ~method_declaration();

  void refine(scope &scp) const;

private:
  const std::vector<std::string> return_type;
  const std::string name;
  const std::vector<std::pair<std::vector<std::string>, std::string>> parameters;
  const std::vector<statement *> statements;
};
}
}