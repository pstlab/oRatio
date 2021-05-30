#pragma once

#include "solver_export.h"
#include "core.h"

#define AT "at"
#define START "start"
#define END "end"

#ifdef BUILD_LISTENERS
#define FIRE_NEW_FLAW(f) fire_new_flaw(f)
#define FIRE_FLAW_COST_CHANGED(f) fire_flaw_cost_changed(f)
#define FIRE_CURRENT_FLAW(f) fire_current_flaw(f)
#define FIRE_NEW_RESOLVER(r) fire_new_resolver(r)
#define FIRE_CURRENT_RESOLVER(r) fire_current_resolver(r)
#define FIRE_CAUSAL_LINK_ADDED(f, r) fire_causal_link_added(f, r)
#else
#define FIRE_NEW_FLAW(f)
#define FIRE_FLAW_COST_CHANGED(f)
#define FIRE_CURRENT_FLAW(f)
#define FIRE_NEW_RESOLVER(r)
#define FIRE_CURRENT_RESOLVER(r)
#define FIRE_CAUSAL_LINK_ADDED(f, r)
#endif

namespace ratio
{
  class heuristic;
  class flaw;
  class resolver;
  class smart_type;
  class atom_flaw;
#ifdef BUILD_LISTENERS
  class solver_listener;
#endif

  class solver : public core, private smt::theory
  {
    friend class heuristic;
    friend class flaw;
    friend class smart_type;
    friend class atom_flaw;
#ifdef BUILD_LISTENERS
    friend class solver_listener;
#endif

  public:
    SOLVER_EXPORT solver();
    solver(const solver &orig) = delete;
    SOLVER_EXPORT ~solver();

    SOLVER_EXPORT void read(const std::string &script) override;
    SOLVER_EXPORT void read(const std::vector<std::string> &files) override;

    /**
     * Initializes the solver.
     */
    SOLVER_EXPORT void init() noexcept;

    /**
     * Solves the given problem.
     */
    SOLVER_EXPORT void solve() override;
    /**
     * Takes the given decision and propagates its effects.
     */
    void take_decision(const smt::lit &ch);

    bool_expr new_bool() noexcept override;
    expr new_enum(const type &tp, const std::vector<item *> &allowed_vals) override;
    bool_expr disj(const std::vector<bool_expr> &exprs) noexcept override;

    inline size_t decision_level() const noexcept { return trail.size(); } // returns the current decision level..
    inline bool root_level() const noexcept { return trail.empty(); }      // checks whether the current decision level is root level..

  private:
    inline atom_flaw &get_reason(const atom &atm) const noexcept { return *reason.at(&atm); } // returns the flaw which has given rise to the atom..
    inline const std::vector<resolver *> get_cause()
    {
      if (res)
        return {res};
      else
        return {};
    }

    void new_atom(atom &atm, const bool &is_fact) override;                                       // notifies the creation of a new atom..
    void new_disjunction(context &d_ctx, const std::vector<const conjunction *> &conjs) override; // notifies the creation of a new disjunction..

    void new_flaw(flaw &f, const bool &enqueue = true); // notifies the solver that a new flaw 'f' has been created..
    void new_resolver(resolver &r);                     // notifies the solver that a new resolver 'r' has been created..
    void new_causal_link(flaw &f, resolver &r);         // notifies the solver that a new causal link between a flaw 'f' and a resolver 'r' has been created..

    void expand_flaw(flaw &f);                         // expands the given flaw..
    void apply_resolver(resolver &r);                  // applies the given resolver..
    void set_cost(flaw &f, const smt::rational &cost); // sets the cost of the given flaw..

    inline std::vector<flaw *> flush_pending_flaws()
    {
      std::vector<flaw *> pndng_flaws;
      std::swap(pndng_flaws, pending_flaws);
      return pndng_flaws;
    }

    void next();

    bool propagate(const smt::lit &p) override;
    bool check() override;
    void push() override;
    void pop() override;

    void solve_inconsistencies();                                     // checks whether the types have any inconsistency and, in case, solve them..
    std::vector<std::vector<std::pair<smt::lit, double>>> get_incs(); // collects all the current inconsistencies..

    void reset_smart_types();

  private:
    struct layer
    {
      layer(const smt::lit &dec) : decision(dec) {}

      const smt::lit decision;                               // the decision which introduced the new layer..
      std::unordered_map<flaw *, smt::rational> old_f_costs; // the old estimated flaws' costs..
      std::unordered_set<flaw *> new_flaws;                  // the just activated flaws..
      std::unordered_set<flaw *> solved_flaws;               // the just solved flaws..
    };
    heuristic &heur;                                            // the used heuristic..
    std::vector<smart_type *> sts;                              // the smart-types..
    resolver *res = nullptr;                                    // the current resolver (i.e. the cause for the new flaws)..
    std::vector<flaw *> pending_flaws;                          // pending flaws, waiting root-level for being initialized..
    std::unordered_map<smt::var, std::vector<flaw *>> phis;     // the phi variables (propositional variable to flaws) of the flaws..
    std::unordered_map<smt::var, std::vector<resolver *>> rhos; // the rho variables (propositional variable to resolver) of the resolvers..
    std::unordered_map<const atom *, atom_flaw *> reason;       // the reason for having introduced an atom..
    std::unordered_set<flaw *> flaws;                           // the currently active flaws..
    smt::lit current_decision;                                  // the decision which has just been taken..
    std::vector<layer> trail;                                   // the list of taken decisions, with the associated changes made, in chronological order..

#ifdef BUILD_LISTENERS
  private:
    std::vector<solver_listener *> listeners; // the solver listeners..

    void fire_new_flaw(const flaw &f) const;
    void fire_flaw_state_changed(const flaw &f) const;
    void fire_flaw_cost_changed(const flaw &f) const;
    void fire_current_flaw(const flaw &f) const;
    void fire_new_resolver(const resolver &r) const;
    void fire_resolver_state_changed(const resolver &r) const;
    void fire_current_resolver(const resolver &r) const;
    void fire_causal_link_added(const flaw &f, const resolver &r) const;
#endif
  };
} // namespace ratio