#pragma once

#include "type_declaration.h"
#include <vector>

namespace lucy
{

namespace ast
{

class field_declaration;
class constructor_declaration;
class method_declaration;
class predicate_declaration;

class class_declaration : public type_declaration
{
public:
  class_declaration(const std::string &n, const std::vector<std::vector<std::string>> &bcs, const std::vector<field_declaration *> &fs, const std::vector<constructor_declaration *> &cs, const std::vector<method_declaration *> &ms, const std::vector<predicate_declaration *> &ps, const std::vector<type_declaration *> &ts);
  class_declaration(const class_declaration &orig) = delete;
  virtual ~class_declaration();

  void declare(scope &scp) const override;
  void refine(scope &scp) const override;

private:
  const std::vector<std::vector<std::string>> base_classes;
  const std::vector<field_declaration *> fields;
  const std::vector<constructor_declaration *> constructors;
  const std::vector<method_declaration *> methods;
  const std::vector<predicate_declaration *> predicates;
  const std::vector<type_declaration *> types;
};
}
}