#include "executor_listener.h"
#include <jni.h>

namespace ratio
{

  class java_executor_listener : public executor_listener
  {
  public:
    java_executor_listener(executor &e, JNIEnv *env, jobject obj);
    ~java_executor_listener();

  private:
    virtual void startedSolving() override;
    virtual void solutionFound() override;
    virtual void inconsistentProblem() override;

    virtual void tick(const smt::rational time) override;
    virtual void starting(const std::set<atom *> &atoms) override;
    virtual void ending(const std::set<atom *> &atoms) override;

  private:
    JNIEnv *env;
    jobject slv_obj;   // the java solver instance..
    jclass solver_cls; // the java solver class..
    jmethodID started_solving_mthd_id, solution_found_mthd_id, inconsistent_problem_mthd_id, tick_mthd_id, starting_mthd_id, ending_mthd_id;
  };
} // namespace ratio
