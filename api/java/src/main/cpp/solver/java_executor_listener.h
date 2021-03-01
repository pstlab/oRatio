#pragma once

#include "scoped_env.h"
#include "executor_listener.h"

namespace ratio
{

  class java_executor_listener : public scoped_env, public executor_listener
  {
  public:
    java_executor_listener(executor &e, JNIEnv *env, jobject obj);
    ~java_executor_listener();

  private:
    virtual void tick(const smt::rational time) override;
    virtual void starting(const std::set<atom *> &atoms) override;
    virtual void ending(const std::set<atom *> &atoms) override;

  private:
    jobject exec_obj; // the java executor instance..
    jclass exec_cls;  // the java executor class..
    jmethodID tick_mthd_id, starting_mthd_id, ending_mthd_id;
  };
} // namespace ratio
