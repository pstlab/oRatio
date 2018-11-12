#pragma once

#include "graph.h"
#include "lit.h"

namespace ratio
{

class atom;

class atom_flaw : public flaw
{
  friend class smart_type;

public:
  atom_flaw(solver &slv, resolver *const cause, atom &a, const bool is_fact);
  atom_flaw(const atom_flaw &orig) = delete;
  virtual ~atom_flaw();

  atom &get_atom() const { return atm; }

private:
  void compute_resolvers() override;

  class activate_fact : public resolver
  {
  public:
    activate_fact(solver &slv, atom_flaw &f, atom &a);
    activate_fact(const activate_fact &that) = delete;
    virtual ~activate_fact();

  private:
    void apply() override;

  private:
    atom &atm;
  };

  class activate_goal : public resolver
  {
  public:
    activate_goal(solver &slv, atom_flaw &f, atom &a);
    activate_goal(const activate_goal &that) = delete;
    virtual ~activate_goal();

  private:
    void apply() override;

  private:
    atom &atm;
  };

  class unify_atom : public resolver
  {
  public:
    unify_atom(solver &slv, atom_flaw &atm_flaw, atom &atm, atom &trgt, const std::vector<smt::lit> &unif_lits);
    unify_atom(const unify_atom &that) = delete;
    virtual ~unify_atom();

  private:
    void apply() override;

  private:
    atom &atm;
    atom &trgt;
    const std::vector<smt::lit> unif_lits;
  };

private:
  atom &atm;

public:
  const bool is_fact;
};
} // namespace ratio
