#pragma once

#include "lit.h"
#include "rational.h"
#include <vector>

namespace ratio
{
  class solver;
  class resolver;

  class flaw
  {
    friend class solver;
    friend class resolver;

  public:
    flaw(solver &slv, const std::vector<resolver *> &causes, const bool &exclusive = false);
    flaw(const flaw &that) = delete;
    ~flaw();

    inline solver &get_solver() const noexcept { return slv; }
    inline smt::lit get_phi() const noexcept { return phi; }
    inline smt::var get_position() const noexcept { return position; }
    inline const std::vector<resolver *> &get_resolvers() const noexcept { return resolvers; }
    inline const std::vector<resolver *> &get_causes() const noexcept { return causes; }
    inline const std::vector<resolver *> &get_supports() const noexcept { return supports; }

    inline smt::rational get_estimated_cost() const noexcept { return est_cost; }
    inline bool is_expanded() const noexcept { return expanded; }

    resolver *get_cheapest_resolver() const noexcept;
    virtual resolver *get_best_resolver() const noexcept { return get_cheapest_resolver(); }

    virtual std::string get_label() const = 0;

  private:
    /**
     * Initializes this flaw.
     * 
     * @pre the solver must be at root-level.
     */
    void init() noexcept;
    /**
     * Expands this flaw, invoking the compute_resolvers procedure.
     * 
     * @pre the solver must be at root-level.
     */
    void expand();
    virtual void compute_resolvers() = 0;

  protected:
    /**
     * Adds the resolver 'r' to this flaw.
     */
    void add_resolver(resolver &r);

  private:
    solver &slv;                                               // the solver this flaw belongs to..
    smt::lit phi;                                              // the propositional literal indicating whether the flaw is active or not (this literal is initialized by the 'init' procedure)..
    smt::var position;                                         // the position variable (i.e., an integer time-point) associated to this flaw..
    smt::rational est_cost = smt::rational::POSITIVE_INFINITY; // the current estimated cost of the flaw..
    bool expanded = false;                                     // a boolean indicating whether the flaw has been expanded..
    std::vector<resolver *> resolvers;                         // the resolvers for this flaw..
    const std::vector<resolver *> causes;                      // the causes for having this flaw (used for activating the flaw through causal propagation)..
    std::vector<resolver *> supports;                          // the resolvers supported by this flaw (used for propagating cost estimates)..
    const bool exclusive;                                      // a boolean indicating whether the flaw is exclusive (i.e. exactly one of its resolver can be applied)..
  };
} // namespace ratio