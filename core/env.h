#pragma once

#include "context.h"
#include "json.h"
#include <map>
#include <string>

namespace ratio
{
  class core;
  class context;
  class constructor;
  class method;
  class predicate;

  namespace ast
  {
    class local_field_statement;
    class assignment_statement;
    class formula_statement;
    class return_statement;
  } // namespace ast

  class env
  {
    friend class context;
    friend class core;
    friend class constructor;
    friend class method;
    friend class predicate;
    friend class item;
    friend class var_item;
    friend class ast::local_field_statement;
    friend class ast::assignment_statement;
    friend class ast::formula_statement;
    friend class ast::return_statement;

  public:
    CORE_EXPORT env(core &cr, const context ctx);
    env(const env &orig) = delete;
    CORE_EXPORT virtual ~env();

    inline core &get_core() const { return cr; } // returns the core in which this environment is created..

    virtual expr get(const std::string &name) const;                                // returns the expression having the given name, checks in the enclosing environment if the name is not found (notice that this method returns a new expression, incrementing the number of references to the expression having the given name)..
    inline std::map<std::string, expr> get_exprs() const noexcept { return exprs; } // returns a map of names and their corresponding expressions directly accessible from this environment..

    virtual smt::json to_json() const noexcept;

  private:
    core &cr;                          // the core in which this environment is created..
    unsigned ref_count;                // the number of references for this environment (used for implementing a 'smart pointer' infrastructure)..
    const context ctx;                 // the context in which this environment was created..
    std::map<std::string, expr> exprs; // the expressions defined within this environment..
  };
} // namespace ratio