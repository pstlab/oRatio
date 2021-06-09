#pragma once

#include "scoped_env.h"
#include "core_listener.h"
#include <unordered_map>
#include <unordered_set>

namespace ratio
{
  class type;
  class predicate;

  class java_core_listener : public scoped_env, public core_listener
  {
  public:
    java_core_listener(core &cr, JNIEnv *env, jobject obj);
    ~java_core_listener();

  private:
    void log(const std::string &msg) override;
    void read(const std::string &script) override;
    void read(const std::vector<std::string> &files) override;

    void state_changed() override;

    void started_solving() override;
    void solution_found() override;
    void inconsistent_problem() override;

    void new_type(JNIEnv *env, const type &t);
    void revise_type(JNIEnv *env, const type &t);
    void new_predicate(JNIEnv *env, const predicate &p);
    void revise_predicate(JNIEnv *env, const predicate &p);

    void new_item(JNIEnv *env, const item &itm);
    void new_atom(JNIEnv *env, const atom &atm);

    jobject new_field(JNIEnv *env, const std::string &name, const type &tp);
    jobjectArray new_fields_array(JNIEnv *env, const std::vector<const field *> &args);
    jobject set(JNIEnv *env, jobject c_obj, jmethodID mthd_id, const std::string &name, const item &itm);

  private:
    jobject slv_obj;   // the java solver instance..
    jclass solver_cls; // the java solver class..
    jmethodID log_mthd_id, read0_mthd_id, read1_mthd_id, state_changed_mthd_id;
    jmethodID s_dfn_field_mthd_id, s_dfn_method_mthd_id, s_dfn_type_mthd_id, s_dfn_pred_mthd_id, s_set_mthd_id;
    jmethodID s_strtd_slvng_mthd_id, s_sol_found_mthd_id, s_inc_prblm_mthd_id;
    jclass type_cls; // the java type class..
    jmethodID type_ctr_id;
    jmethodID t_dfn_constructor_mthd_id, t_dfn_superclass_mthd_id, t_dfn_field_mthd_id, t_dfn_method_mthd_id, t_dfn_type_mthd_id, t_dfn_pred_mthd_id, t_new_instnc;
    jclass ctr_cls;             // the java constructor class..
    jmethodID ctr_ctr_id;       // the constructor constructor..
    jclass mthd_cls;            // the java method class..
    jmethodID mthd_ctr_id;      // the method constructor..
    jclass field_cls;           // the java field class..
    jmethodID field_ctr_id;     // the field constructor..
    jclass predicate_cls;       // the java predicate class..
    jmethodID predicate_ctr_id; // the predicate constructor..
    jclass item_cls;            // the java item class..
    jmethodID item_ctr_id;      // the item constructor..
    jmethodID i_set_name_mthd_id, i_set_mthd_id;
    jclass bool_item_cls;       // the java bool item class..
    jmethodID bool_item_ctr_id; // the bool item constructor..
    jmethodID bool_item_set_mthd_id;
    jclass rat_cls;              // the java rational class..
    jmethodID rat_ctr_id;        // the rational constructor..
    jclass inf_rat_cls;          // the java inf rational class..
    jmethodID inf_rat_ctr_id;    // the inf rational constructor..
    jclass arith_item_cls;       // the java arith item class..
    jmethodID arith_item_ctr_id; // the bool arith constructor..
    jmethodID arith_item_set_lb_mthd_id, arith_item_set_ub_mthd_id, arith_item_set_val_mthd_id;
    jclass enum_item_cls;       // the java enum item class..
    jmethodID enum_item_ctr_id; // the enum item constructor..
    jmethodID enum_item_set_mthd_id;
    jclass string_item_cls;       // the java string item class..
    jmethodID string_item_ctr_id; // the string item constructor..
    jmethodID string_item_set_mthd_id;
    jclass atm_cls;       // the java atom class..
    jmethodID atm_ctr_id; // the atom constructor..
    jmethodID atm_set_state_mthd_id;
    std::unordered_map<const type *, jobject> all_types;
    std::unordered_map<const item *, jobject> all_items;
  };
} // namespace ratio
