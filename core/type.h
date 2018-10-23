#pragma once

#include "scope.h"

namespace ratio
{

class type : public scope
{
public:
  type(core &cr, scope &scp, const std::string &name, bool primitive = false);
  type(const type &orig) = delete;
  virtual ~type();

  std::string get_name() { return name; }
  bool is_primitive() { return primitive; }

private:
  const std::string name; // the name of this type..
  const bool primitive;   // is this type a primitive type?
};
} // namespace ratio
