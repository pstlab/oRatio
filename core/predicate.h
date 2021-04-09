#pragma once

#include "type.h"

namespace riddle::ast
{
  class statement;
} // namespace riddle::ast

namespace ratio
{
  class expr;
  class atom;

  class predicate : public type
  {
  public:
    CORE_EXPORT predicate(core &cr, scope &scp, const std::string &name, const std::vector<const field *> &args, const std::vector<const riddle::ast::statement *> &stmnts);
    predicate(const predicate &orig) = delete;
    CORE_EXPORT virtual ~predicate();

    inline const std::vector<const field *> get_args() const noexcept { return args; }

    CORE_EXPORT expr new_instance(context &ctx) noexcept override; // creates a new atom having this predicate within the given context..

    CORE_EXPORT void apply_rule(atom &a) const; // applies the rule associated to this predicate to the given atom..

  private:
    const std::vector<const field *> args;                        // the arguments of the predicate..
    const std::vector<const riddle::ast::statement *> statements; // the statements constituting the rule's body..
  };
} // namespace ratio