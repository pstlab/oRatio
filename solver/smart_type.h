#pragma once

#include "type.h"
#include "graph.h"
#include "sat_value_listener.h"
#include "lra_value_listener.h"
#include "ov_value_listener.h"

namespace ratio
{

class solver;
class graph;
class atom;
class flaw;
class atom_flaw;
class resolver;

class smart_type : public type
{
  friend class solver;
  friend class graph;

public:
  smart_type(solver &slv, scope &scp, const std::string &name);
  smart_type(const smart_type &that) = delete;
  virtual ~smart_type();

  solver &get_solver() const { return slv; }

private:
  /**
   * Returns all the decisions to take for solving the current inconsistencies with their choices' estimated costs.
   * 
   * @return a vector of decisions, each represented by a vector of pairs containing the literals representing the choice and its estimated cost.
   */
  virtual std::vector<std::vector<std::pair<smt::lit, double>>> get_current_incs() = 0;

  virtual void new_atom(atom_flaw &);

protected:
  void set_ni(const smt::var &v); // temporally sets the solver's 'ni' variable..
  void restore_ni();              // restores the solver's 'ni' variable..

  void store_flaw(flaw &f); // stores the flaw waiting for its initialization at root-level..

  static std::vector<resolver *> get_resolvers(solver &slv, const std::set<atom *> &atms); // returns the vector of resolvers which has given rise to the given atoms..

private:
  solver &slv;
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
