#pragma once

#include "flaw.h"
#include "resolver.h"
#include "lit.h"

namespace ratio
{

class atom;

class atom_flaw : public flaw
{
  friend class smart_type;
  friend class graph;

public:
  atom_flaw(graph &gr, resolver *const cause, atom &a, const bool is_fact);
  atom_flaw(const atom_flaw &orig) = delete;
  virtual ~atom_flaw();

  atom &get_atom() const { return atm; }

#ifdef BUILD_GUI
  std::string get_label() const override;
#endif

private:
  void compute_resolvers() override;

  class activate_fact : public resolver
  {
  public:
    activate_fact(graph &gr, atom_flaw &f, atom &a);
    activate_fact(graph &gr, const smt::var &r, atom_flaw &f, atom &a);
    activate_fact(const activate_fact &that) = delete;
    virtual ~activate_fact();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

  private:
    void apply() override;

  private:
    atom &atm; // the fact which will be activated..
  };

  class activate_goal : public resolver
  {
  public:
    activate_goal(graph &gr, atom_flaw &f, atom &a);
    activate_goal(graph &gr, const smt::var &r, atom_flaw &f, atom &a);
    activate_goal(const activate_goal &that) = delete;
    virtual ~activate_goal();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

  private:
    void apply() override;

  private:
    atom &atm; // the goal which will be activated..
  };

  class unify_atom : public resolver
  {
  public:
    unify_atom(graph &gr, atom_flaw &atm_flaw, atom &atm, atom &trgt, const std::vector<smt::lit> &unif_lits);
    unify_atom(const unify_atom &that) = delete;
    virtual ~unify_atom();

#ifdef BUILD_GUI
    std::string get_label() const override;
#endif

  private:
    void apply() override;

  private:
    atom &atm;                             // the unifying atom..
    atom &trgt;                            // the target atom..
    const std::vector<smt::lit> unif_lits; // the unification literals..
  };

private:
  atom &atm; // the atom which has to be justified..

public:
  const bool is_fact;
};
} // namespace ratio