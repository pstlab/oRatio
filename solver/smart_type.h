#pragma once

#include "type.h"
#include "solver.h"
#include "atom_flaw.h"
#include "field.h"
#include "sat_value_listener.h"
#include "lra_value_listener.h"
#include "ov_value_listener.h"

namespace ratio
{

class flaw;

class smart_type : public type
{
  friend class solver;

public:
  smart_type(solver &slv, scope &scp, const std::string &name) : type(slv, scp, name, false), slv(slv) {}
  smart_type(const smart_type &that) = delete;

  virtual ~smart_type() {}

private:
  virtual std::vector<flaw *> get_flaws() = 0;
  virtual void new_fact(atom_flaw &) {}
  virtual void new_goal(atom_flaw &) {}

public:
  static inline std::vector<resolver *> get_resolvers(solver &slv, const std::set<atom *> &atms) // returns the vector of resolvers which has given rise to the given atoms..
  {
    std::unordered_set<resolver *> ress;
    for (const auto &atm : atms)
      for (const auto &r : slv.get_flaw(*atm).get_resolvers())
        if (resolver *af = dynamic_cast<atom_flaw::activate_fact *>(r))
          ress.insert(af);
        else if (resolver *ag = dynamic_cast<atom_flaw::activate_goal *>(r))
          ress.insert(ag);

    return std::vector<resolver *>(ress.begin(), ress.end());
  }

protected:
  solver &slv;
};

class atom_listener : public smt::sat_value_listener, public smt::lra_value_listener, public smt::ov_value_listener
{
public:
  atom_listener(atom &atm) : smt::sat_value_listener(atm.get_core().sat_cr), smt::lra_value_listener(atm.get_core().la_th), smt::ov_value_listener(atm.get_core().ov_th), atm(atm)
  {
    std::queue<const type *> q;
    q.push(&atm.tp);
    while (!q.empty())
    {
      for (const auto &f : q.front()->get_fields())
        if (!f.second->synthetic)
        {
          item *i = &*atm.get(f.first);
          if (bool_item *be = dynamic_cast<bool_item *>(i))
            listen_sat(be->l.v);
          else if (arith_item *ae = dynamic_cast<arith_item *>(i))
            for (const auto &term : ae->l.vars)
              listen_lra(term.first);
          else if (var_item *ee = dynamic_cast<var_item *>(i))
            listen_set(ee->ev);
        }

      for (const auto &st : q.front()->get_supertypes())
        q.push(st);
      q.pop();
    }
  }
  atom_listener(const atom_listener &that) = delete;

  virtual ~atom_listener() {}

protected:
  atom &atm;
};
}