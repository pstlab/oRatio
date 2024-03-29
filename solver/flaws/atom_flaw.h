#pragma once

#include "flaw.h"
#include "resolver.h"
#include "lit.h"

namespace ratio
{
  class atom;
  class smart_type;

  bool is_unification(const resolver &r);

  class atom_flaw final : public flaw
  {
    friend class smart_type;

  public:
    atom_flaw(solver &slv, std::vector<resolver *> causes, atom &a, const bool is_fact);
    atom_flaw(const atom_flaw &orig) = delete;

    inline atom &get_atom() const noexcept { return atm; }

    std::string get_data() const noexcept override;

  private:
    void compute_resolvers() override;

  private:
    class activate_fact final : public resolver
    {
    public:
      activate_fact(atom_flaw &f, atom &a);
      activate_fact(const smt::lit &r, atom_flaw &f, atom &a);
      activate_fact(const activate_fact &that) = delete;

      std::string get_data() const noexcept override;

    private:
      void apply() override;

    private:
      atom &atm; // the fact which will be activated..
    };

    class activate_goal final : public resolver
    {
    public:
      activate_goal(atom_flaw &f, atom &a);
      activate_goal(const smt::lit &r, atom_flaw &f, atom &a);
      activate_goal(const activate_goal &that) = delete;

      std::string get_data() const noexcept override;

    private:
      void apply() override;

    private:
      atom &atm; // the goal which will be activated..
    };

    class unify_atom final : public resolver
    {
    public:
      unify_atom(atom_flaw &atm_flaw, atom &atm, atom &trgt, const std::vector<smt::lit> &unif_lits);
      unify_atom(const unify_atom &that) = delete;

      std::string get_data() const noexcept override;

    private:
      void apply() override;

    private:
      atom &atm;                             // the unifying atom..
      atom &trgt;                            // the target atom..
      const std::vector<smt::lit> unif_lits; // the unification literals..
    };

    friend inline bool is_unification(const resolver &r) { return dynamic_cast<const unify_atom *>(&r); }

  private:
    atom &atm; // the atom which has to be justified..

  public:
    const bool is_fact;
  };
} // namespace ratio