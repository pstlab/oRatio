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

  field &get_field(const std::string &name) const override; // returns the field having the given name..

  std::string get_name() const { return name; }
  bool is_primitive() const { return primitive; }

protected:
  static void new_supertypes(type &t, const std::vector<type *> &sts)
  {
    for (const auto &st : sts)
      t.supertypes.push_back(st);
  }

private:
  const std::string name;         // the name of this type..
  const bool primitive;           // is this type a primitive type?
  std::vector<type *> supertypes; // the base types (i.e. the types this type inherits from)..
};
} // namespace ratio
