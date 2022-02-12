#pragma once

#include "solver_listener.h"

namespace ratio
{

  class gui_solver_listener : public solver_listener
  {
  public:
    gui_solver_listener(solver &s);
    ~gui_solver_listener();

  private:
    void flaw_created(const flaw &f) override;
    void flaw_state_changed(const flaw &f) override;
    void flaw_cost_changed(const flaw &f) override;
    void flaw_position_changed(const flaw &f) override;
    void current_flaw(const flaw &f) override;

    void resolver_created(const resolver &r) override;
    void resolver_state_changed(const resolver &r) override;
    void current_resolver(const resolver &r) override;

    void causal_link_added(const flaw &f, const resolver &r) override;
  };
} // namespace ratio