#pragma once

#include "flaw.h"
#include "resolver.h"
#include "lit.h"

namespace ratio
{
  class atom;
  class smart_type;

  class atom_flaw : public flaw
  {
    friend class smart_type;

  public:
    atom_flaw(solver &slv, const std::vector<resolver *> &causes, atom &a, const bool is_fact);
    atom_flaw(const atom_flaw &orig) = delete;
    virtual ~atom_flaw();

    inline atom &get_atom() const noexcept { return atm; }

    std::string get_label() const noexcept override;

  private:
    void compute_resolvers() override;

  private:
    class activate_fact : public resolver
    {
    public:
      activate_fact(solver &slv, atom_flaw &f, atom &a);
      activate_fact(solver &slv, const smt::lit &r, atom_flaw &f, atom &a);
      activate_fact(const activate_fact &that) = delete;
      virtual ~activate_fact();

      std::string get_label() const noexcept override;

    private:
      void apply() override;

    private:
      atom &atm; // the fact which will be activated..
    };

    class activate_goal : public resolver
    {
    public:
      activate_goal(solver &slv, atom_flaw &f, atom &a);
      activate_goal(solver &slv, const smt::lit &r, atom_flaw &f, atom &a);
      activate_goal(const activate_goal &that) = delete;
      virtual ~activate_goal();

      std::string get_label() const noexcept override;

    private:
      void apply() override;

    private:
      atom &atm; // the goal which will be activated..
    };

    class unify_atom : public resolver
    {
    public:
      unify_atom(solver &slv, atom_flaw &atm_flaw, atom &atm, atom &trgt, const std::vector<smt::lit> &unif_lits);
      unify_atom(const unify_atom &that) = delete;
      virtual ~unify_atom();

      std::string get_label() const noexcept override;

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