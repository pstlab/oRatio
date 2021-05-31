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
    void tick(const smt::rational time) override;
    void starting(const std::unordered_set<atom *> &atoms) override;
    void start(const std::unordered_set<atom *> &atoms) override;
    void ending(const std::unordered_set<atom *> &atoms) override;
    void end(const std::unordered_set<atom *> &atoms) override;

    void call_method(const jmethodID &mthd_id, const std::unordered_set<atom *> &atoms);

  private:
    jobject exec_obj; // the java executor instance..
    jclass exec_cls;  // the java executor class..
    jmethodID tick_mthd_id, starting_mthd_id, start_mthd_id, ending_mthd_id, end_mthd_id;
  };
} // namespace ratio
