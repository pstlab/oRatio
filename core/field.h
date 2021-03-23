#pragma once

#include <string>

namespace riddle::ast
{
  class expression;
} // namespace riddle::ast

namespace ratio
{
  class type;

  class field
  {
  public:
    field(const type &tp, const std::string &name, const riddle::ast::expression *const e = nullptr, bool synthetic = false) : tp(tp), name(name), xpr(e), synthetic(synthetic) {}
    virtual ~field() {}

    inline const type &get_type() const { return tp; }                           // returns the type of the field..
    inline const std::string &get_name() const { return name; }                  // returns the name of the field..
    inline const riddle::ast::expression *get_expression() const { return xpr; } // returns, if any, the initialization expression..
    inline bool is_synthetic() const { return synthetic; }                       // returns whether the field is synthetic or not..

  private:
    const type &tp;                           // the type of the field..
    const std::string name;                   // the name of the field..
    const riddle::ast::expression *const xpr; // the initialization expression..
    const bool synthetic;                     // the field is synthetic (a synthetic field is a field which is not created by the user, e.g. 'this')..
  };
} // namespace ratio