#pragma once

#include "sat_core.h"
#include "theory.h"
#include "var_value.h"
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <set>

namespace smt
{
  class ov_value_listener;

  class ov_theory : public theory
  {
    friend class ov_value_listener;

  public:
    SMT_EXPORT ov_theory(sat_core &sat);
    ov_theory(const ov_theory &orig) = delete;
    SMT_EXPORT virtual ~ov_theory();

    SMT_EXPORT var new_var(const std::vector<var_value *> &items, const bool enforce_exct_one = true) noexcept; // creates and returns a new object variable having the given domain..
    SMT_EXPORT var new_var(const std::vector<lit> &lits, const std::vector<var_value *> &vals) noexcept;        // creates and returns a new object variable having the given domain, the presence of the values into the domain is controlled by the 'lits' literals..

    SMT_EXPORT lit allows(const var &v, const var_value &val) const noexcept; // returns the literal controlling the presence of the 'val' value into the domain of variable 'v'..
    SMT_EXPORT lit new_eq(const var &left, const var &right) noexcept;        // creates an equality constraints between 'left' and 'right' variables returning the literal that controls it..

    SMT_EXPORT std::unordered_set<const var_value *> value(var v) const noexcept; // returns the current domain of the object variable 'v'..

  private:
    bool propagate(const lit &p) noexcept override;
    bool check() noexcept override;
    void push() noexcept override;
    void pop() noexcept override;

    inline void listen(const var &v, ov_value_listener *const l) noexcept { listening[v].insert(l); }

  private:
    struct layer
    {
      std::unordered_set<var> vars; // the updated variables..
    };
    std::vector<std::unordered_map<const var_value *, lit>> assigns; // the current assignments (val to literal)..
    std::unordered_map<std::string, lit> exprs;                      // the already existing expressions (string to literal)..
    std::unordered_map<var, std::set<var>> is_contained_in;          // the propositional variable contained in the object variables (bool variable to object variables)..
    std::vector<layer> layers;                                       // we store the updated variables..
    std::unordered_map<var, std::set<ov_value_listener *>> listening;
  };
} // namespace smt