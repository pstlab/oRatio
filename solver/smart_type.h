#pragma once

#include "type.h"
#include "sat_value_listener.h"
#include "lra_value_listener.h"
#include "ov_value_listener.h"

namespace ratio
{

class solver;
class atom;

class smart_type : public type
{
  friend class solver;

public:
  smart_type(solver &slv, scope &scp, const std::string &name);
  smart_type(const smart_type &that) = delete;
  virtual ~smart_type();

  solver &get_solver() const { return slv; }

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
