#pragma once

#include "scoped_env.h"
#include "solver_listener.h"

namespace ratio
{

  class java_solver_listener : public scoped_env, public solver_listener
  {
  public:
    java_solver_listener(solver &s, JNIEnv *env, jobject obj);
    ~java_solver_listener();

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

  private:
    jobject slv_obj;   // the java solver instance..
    jclass solver_cls; // the java solver class..
    jmethodID flaw_created_mthd_id, flaw_state_changed_mthd_id, flaw_cost_changed_mthd_id, flaw_position_changed_mthd_id, current_flaw_mthd_id, resolver_created_mthd_id, resolver_state_changed_mthd_id, current_resolver_mthd_id, causal_link_added_mthd_id;
  };
} // namespace ratio
