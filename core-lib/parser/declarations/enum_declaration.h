#pragma once

#include "type_declaration.h"
#include <vector>

namespace lucy
{

namespace ast
{

class enum_declaration : public type_declaration
{
public:
  enum_declaration(const std::string &n, const std::vector<std::string> &es, const std::vector<std::vector<std::string>> &trs);
  enum_declaration(const enum_declaration &orig) = delete;
  virtual ~enum_declaration();

  void declare(scope &scp) const override;
  void refine(scope &scp) const override;

private:
  const std::vector<std::string> enums;
  const std::vector<std::vector<std::string>> type_refs;
};
}
}