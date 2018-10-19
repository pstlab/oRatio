#pragma once

#include "sat_core.h"
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
  ov_theory(sat_core &sat);
  ov_theory(const ov_theory &orig) = delete;
  virtual ~ov_theory();

  var new_var(const std::unordered_set<var_value *> &items);                       // creates and returns a new object variable having the given domain..
  var new_var(const std::vector<var> &vars, const std::vector<var_value *> &vals); // creates and returns a new object variable having the given domain, the presence of the values into the domain is controlled by the 'vars' propositional variables..

  var allows(const var &v, var_value &val) const;                      // returns the propositional variable that controls the presence of the 'val' value into the domain of variable 'v'..
  var new_eq(const var &left, const var &right);                       // creates an equality constraints between 'left' and 'right' variables returning a propositional variable that controls it..
  bool eq(const var &left, const var &right, const var &p = TRUE_var); // creates an equality constraints between 'left' and 'right' variables with 'p' propositional variable that controls it..

  std::unordered_set<var_value *> value(var v) const; // returns the current domain of the object variable 'v'..

private:
  bool propagate(const lit &p, std::vector<lit> &cnfl) override;
  bool check(std::vector<lit> &cnfl) override;
  void push() override;
  void pop() override;

  void listen(const var &v, ov_value_listener *const l);

private:
  class layer
  {
  public:
    layer() {}

  public:
    // the updated variables..
    std::unordered_set<var> vars;
  };
  std::vector<std::unordered_map<var_value *, var>> assigns; // the current assignments (val to bool variable)..
  std::unordered_map<std::string, var> exprs;                // the already existing expressions (string to bool variable)..
  std::unordered_map<var, std::set<var>> is_contained_in;    // the boolean variable contained in the set variables (bool variable to vector of set variables)..
  std::vector<layer> layers;                                 // we store the updated variables..
  std::unordered_map<var, std::set<ov_value_listener *>> listening;
};
} // namespace smt