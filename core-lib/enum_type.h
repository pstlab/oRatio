#pragma once

#include "type.h"
#include <unordered_set>

namespace ratio
{

class item;

namespace ast
{
class enum_declaration;
}

class enum_type : public type
{
  friend class ast::enum_declaration;

public:
  enum_type(core &cr, scope &scp, std::string name);
  enum_type(const enum_type &orig) = delete;
  virtual ~enum_type();

  expr new_instance(context &ctx) override;

private:
  std::unordered_set<item *> get_all_instances() const;

private:
  std::vector<enum_type *> enums;
};
}