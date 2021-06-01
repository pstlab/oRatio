#include "it_cnr_istc_pst_oratio_timelines_TimelinesExecutor.h"
#include "java_executor_listener.h"
#include "timer.h"

using namespace ratio;
using namespace smt;

inline solver *get_solver(JNIEnv *env, jobject obj)
{
    jobject slv_obj = env->GetObjectField(obj, env->GetFieldID(env->GetObjectClass(obj), "solver", "Lit/cnr/istc/pst/oratio/Solver;"));
    return reinterpret_cast<solver *>(env->GetLongField(slv_obj, env->GetFieldID(env->GetObjectClass(slv_obj), "native_handle", "J")));
}

inline executor *get_executor(JNIEnv *env, jobject obj) { return reinterpret_cast<executor *>(env->GetLongField(obj, env->GetFieldID(env->GetObjectClass(obj), "native_handle", "J"))); }

JNIEXPORT jlong JNICALL Java_it_cnr_istc_pst_oratio_timelines_TimelinesExecutor_new_1instance(JNIEnv *env, jobject obj, jstring conf, jlong units_per_tick_num, jlong units_per_tick_den)
{
    solver *s = get_solver(env, obj);
    executor *exec = new executor(*s, env->GetStringUTFChars(conf, JNI_FALSE), rational(static_cast<I>(units_per_tick_num), static_cast<I>(units_per_tick_den)));
    java_executor_listener *jel = new java_executor_listener(*exec, env, obj);
    return reinterpret_cast<jlong>(exec);
}

JNIEXPORT void JNICALL Java_it_cnr_istc_pst_oratio_timelines_TimelinesExecutor_dispose(JNIEnv *env, jobject obj)
{
    delete get_executor(env, obj);
    env->SetLongField(obj, env->GetFieldID(env->GetObjectClass(obj), "native_handle", "J"), 0);
}

JNIEXPORT void JNICALL Java_it_cnr_istc_pst_oratio_timelines_TimelinesExecutor_tick(JNIEnv *env, jobject obj)
{
    try
    {
        get_executor(env, obj)->tick();
    }
    catch (const std::exception &e)
    {
        env->ThrowNew(env->FindClass("it/cnr/istc/pst/oratio/timelines/ExecutorException"), e.what());
    }
}

JNIEXPORT void JNICALL Java_it_cnr_istc_pst_oratio_timelines_TimelinesExecutor_dont_1start_1yet(JNIEnv *env, jobject obj, jlongArray atoms)
{
    const jsize atms_size = env->GetArrayLength(atoms);
    std::vector<jlong> input(atms_size);
    env->GetLongArrayRegion(atoms, 0, atms_size, input.data());

    auto &exec = *get_executor(env, obj);
    std::unordered_set<atom *> atms;
    for (jsize i = 0; i < atms_size; i++)
        atms.insert(&exec.get_atom(input[i]));

    exec.dont_start_yet(atms);
}

JNIEXPORT void JNICALL Java_it_cnr_istc_pst_oratio_timelines_TimelinesExecutor_dont_1end_1yet(JNIEnv *env, jobject obj, jlongArray atoms)
{
    const jsize atms_size = env->GetArrayLength(atoms);
    std::vector<jlong> input(atms_size);
    env->GetLongArrayRegion(atoms, 0, atms_size, input.data());

    auto &exec = *get_executor(env, obj);
    std::unordered_set<atom *> atms;
    for (jsize i = 0; i < atms_size; i++)
        atms.insert(&exec.get_atom(input[i]));

    exec.dont_start_yet(atms);
}

JNIEXPORT void JNICALL Java_it_cnr_istc_pst_oratio_timelines_TimelinesExecutor_failure(JNIEnv *env, jobject obj, jlongArray atoms)
{
    const jsize atms_size = env->GetArrayLength(atoms);
    std::vector<jlong> input(atms_size);
    env->GetLongArrayRegion(atoms, 0, atms_size, input.data());

    auto &exec = *get_executor(env, obj);
    std::unordered_set<atom *> atms;
    for (jsize i = 0; i < atms_size; i++)
        atms.insert(&exec.get_atom(input[i]));

    try
    {
        exec.failure(atms);
    }
    catch (const std::exception &e)
    {
        env->ThrowNew(env->FindClass("it/cnr/istc/pst/oratio/timelines/ExecutorException"), e.what());
    }
}