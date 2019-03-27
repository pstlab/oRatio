#pragma once

#include "type.h"
#include "sat_value_listener.h"
#include "lra_value_listener.h"
#include "ov_value_listener.h"

namespace ratio
{

class solver;
class atom;
class flaw;
class atom_flaw;
class resolver;

class smart_type : public type
{
  friend class solver;

public:
  smart_type(solver &slv, scope &scp, const std::string &name);
  smart_type(const smart_type &that) = delete;
  virtual ~smart_type();

  solver &get_solver() const { return slv; }

private:
  /**
   * Retrieves, and removes, all the flaws found so far.
   * 
   * @return a vector containing pointers to the flaws found so far.
   * @pre the solver must be at root-level.
   */
  std::vector<flaw *> get_flaws();
  /**
   * Returns all the decisions to take for solving the current inconsistencies with their estimated cost.
   * 
   * @return a vector of pairs containing the literals representing the decisions to take and their estimated cost.
   */
  virtual std::vector<std::pair<smt::lit, double>> get_current_incs() = 0;

  virtual void new_fact(atom_flaw &);
  virtual void new_goal(atom_flaw &);

protected:
  void add_flaw(flaw &fl) { flaws.push_back(&fl); }

private:
  solver &slv;
  std::vector<flaw *> flaws;
};

class atom_listener : public smt::sat_value_listener, public smt::lra_value_listener, public smt::ov_value_listener
{
public:
  atom_listener(atom &atm);
  atom_listener(const atom_listener &that) = delete;
  virtual ~atom_listener();

protected:
  atom &atm;
};
} // namespace ratio
