#pragma once

#include <string>

namespace ratio
{

class type;

class field
{
public:
  field(const type &tp, const std::string &name, bool synthetic = false) : tp(tp), name(name), synthetic(synthetic) {}
  virtual ~field() {}

  const type &get_type() const { return tp; }          // returns the type of the field..
  const std::string &get_name() const { return name; } // returns the name of the field..
  bool is_synthetic() const { return synthetic; }      // returns whether the field is synthetic..

private:
  const type &tp;         // the type of the field..
  const std::string name; // the name of the field..
  const bool synthetic;   // the field is synthetic (a synthetic field is a field which is not created by the user, e.g. 'this')..
};
} // namespace ratio
