#pragma once

#include "type.h"
#include "graph.h"
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
   * Returns all the decisions to take for solving the current inconsistencies with their choices' estimated costs.
   * 
   * @return a vector of decisions, each represented by a vector of pairs containing the literals representing the choice and its estimated cost.
   */
  virtual std::vector<std::vector<std::pair<smt::lit, double>>> get_current_incs() = 0;

  virtual void new_fact(atom_flaw &);
  virtual void new_goal(atom_flaw &);

  // the flaw (i.e. two or more inconsistent atoms) that can arise from a smart-type..
  class smart_flaw : public flaw
  {
  public:
    smart_flaw(smart_type &st, const std::set<atom *> &atms);
    smart_flaw(smart_flaw &&) = delete;
    virtual ~smart_flaw();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

  private:
    void compute_resolvers() override;

  private:
    smart_type &st;
    const std::set<atom *> atms;
  };

  // a resolver for solving smart-flaws..
  class smart_resolver : public resolver
  {
  public:
#ifdef BUILD_GUI
    smart_resolver(smart_flaw &flw, const smt::var &r, const std::string &lbl);
#else
    smart_resolver(smart_flaw &flw, const smt::var &r);
#endif
    smart_resolver(const smart_resolver &that) = delete;
    virtual ~smart_resolver();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

  private:
    void apply() override;

#ifdef BUILD_GUI
    const std::string &lbl;
#endif
  };

protected:
  void set_ni(const smt::var &v); // temporally sets the solver's 'ni' variable..
  void restore_ni();              // restores the solver's 'ni' variable..

#ifdef BUILD_GUI
  void set_choices(atom &atm0, atom &atm1, const std::vector<std::pair<smt::lit, std::string>> &chs); // sets the choices for solving an inconsistency between the two atoms..
#else
  void set_choices(atom &atm0, atom &atm1, const std::vector<smt::lit> &chs); // sets the choices for solving an inconsistency between the two atoms..
#endif
  void new_inc(const std::set<atom *> &atms); // a new inconsistency among the given atoms has been found..

  static std::vector<resolver *> get_resolvers(solver &slv, const std::set<atom *> &atms); // returns the vector of resolvers which has given rise to the given atoms..

private:
  solver &slv;
  std::vector<flaw *> flaws;

  std::map<const std::set<atom *>, smart_flaw *> smart_flaws; // the smart-flaws found so far..
#ifdef BUILD_GUI
  std::map<const std::set<atom *>, const std::vector<std::pair<smt::lit, std::string>>> choices; // all the possible choices for solving a flaw with a string for describing the choice..
#else
  std::map<const std::set<atom *>, const std::vector<smt::lit>> choices;      // all the possible choices for solving a flaw..
#endif
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
