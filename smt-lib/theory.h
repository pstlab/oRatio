#pragma once

#include "lit.h"
#include <vector>

namespace smt
{

class sat_core;

class theory
{
    friend class sat_core;

  public:
    theory(sat_core &sat);
    theory(const theory &orig) = delete;
    virtual ~theory();

    sat_core &get_core() const { return sat; }

  protected:
    void bind(const var &v);
    void record(const std::vector<lit> &clause);

  private:
    /**
     * Asks the theory to perform propagation after the given literal has been assigned. Returns true if the propagation succeeds or false if an inconsistency is found. In case of inconsistency, the confl vector is filled with the conflicting constraint.
     *
     * @param p the literal that has been assigned.
     * @param cnfl the vector of literals representing the conflicting constraint.
     * @return true if propagation succeeds or false if an inconsistency is found.
     */
    virtual bool propagate(const lit &p, std::vector<lit> &cnfl) = 0;

    /**
     * Checks whether the theory is consistent with the given propositional assignments. Returns true if the theory is consistent or false if an inconsistency is found. In case of inconsistency, the confl vector is filled with the conflicting constraint.
     *
     * @param cnfl the vector of literals representing the conflicting constraint.
     * @return true if the theory is consistent or false if an inconsistency is found.
     */
    virtual bool check(std::vector<lit> &cnfl) = 0;

    /**
     * Notifies the theory that some information for subsequent backtracking might need to be stored.
     */
    virtual void push() = 0;

    /**
     * Notifies the theory that a backtracking step is required.
     */
    virtual void pop() = 0;

  protected:
    sat_core &sat;
};
}