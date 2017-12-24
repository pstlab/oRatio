#pragma once

#include <vector>

namespace ratio
{

class scope;
class context;

namespace ast
{

class method_declaration;
class predicate_declaration;
class type_declaration;
class statement;

class compilation_unit
{
public:
  compilation_unit(const std::vector<method_declaration *> &ms, const std::vector<predicate_declaration *> &ps, const std::vector<type_declaration *> &ts, const std::vector<statement *> &stmnts);
  compilation_unit(const compilation_unit &orig) = delete;
  virtual ~compilation_unit();

  void declare(scope &scp) const;
  void refine(scope &scp) const;
  void execute(const scope &scp, context &ctx) const;

private:
  const std::vector<method_declaration *> methods;
  const std::vector<predicate_declaration *> predicates;
  const std::vector<type_declaration *> types;
  const std::vector<statement *> statements;
};
}
}