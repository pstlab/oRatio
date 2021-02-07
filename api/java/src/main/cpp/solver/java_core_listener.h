#pragma once

#include "core_listener.h"
#include <jni.h>
#include <unordered_map>

namespace ratio
{

  class java_core_listener : public core_listener
  {
  public:
    java_core_listener(core &cr, JNIEnv *env, jobject obj);
    ~java_core_listener();

  private:
    void log(const std::string &msg) override;
    void read(const std::string &script) override;
    void read(const std::vector<std::string> &files) override;

    void state_changed() override;

  private:
    JNIEnv *env;
    jobject slv_obj;   // the java solver instance..
    jclass solver_cls; // the java solver class..
    jmethodID log_mthd_id, read0_mthd_id, read1_mthd_id, state_changed_mthd_id;
    jmethodID s_dfn_field_mthd_id, s_dfn_method_mthd_id, s_dfn_type_mthd_id, s_dfn_pred_mthd_id, s_set_mthd_id;
    jclass type_cls; // the java type class..
    jmethodID type_ctr_id;
    jmethodID t_dfn_constructor_mthd_id, t_dfn_superclass_mthd_id, t_dfn_field_mthd_id, t_dfn_method_mthd_id, t_dfn_type_mthd_id, t_dfn_pred_mthd_id;
    jclass field_cls;           // the java field class..
    jmethodID field_ctr_id;     // the field constructor..
    jclass predicate_cls;       // the java predicate class..
    jmethodID predicate_ctr_id; // the predicate constructor..
    jclass item_cls;            // the java item class..
    jmethodID item_ctr_id;      // the item constructor..
    jmethodID i_set_mthd_id;
    std::unordered_map<jlong, jobject> all_types;
    std::unordered_map<jlong, jobject> all_items;
  };
} // namespace ratio
