#include "it_cnr_istc_pst_oratio_Solver.h"
#include "java_core_listener.h"
#include "java_solver_listener.h"

using namespace ratio;

inline solver *get_solver(JNIEnv *env, jobject obj) { return reinterpret_cast<solver *>(env->GetLongField(obj, env->GetFieldID(env->GetObjectClass(obj), "native_handle", "J"))); }

JNIEXPORT jlong JNICALL Java_it_cnr_istc_pst_oratio_Solver_new_1instance(JNIEnv *env, jobject obj)
{
    solver *s = new solver();
    java_core_listener *jcl = new java_core_listener(*s, env, obj);
    java_solver_listener *jsl = new java_solver_listener(*s, env, obj);
    s->init();
    return reinterpret_cast<jlong>(s);
}

JNIEXPORT void JNICALL Java_it_cnr_istc_pst_oratio_Solver_dispose(JNIEnv *env, jobject obj)
{
    delete get_solver(env, obj);
    env->SetLongField(obj, env->GetFieldID(env->GetObjectClass(obj), "native_handle", "J"), 0);
}

JNIEXPORT void JNICALL Java_it_cnr_istc_pst_oratio_Solver_read__Ljava_lang_String_2(JNIEnv *env, jobject obj, jstring script)
{
    jboolean is_copy;
    const char *utf_script = env->GetStringUTFChars(script, &is_copy);
    try
    {
        get_solver(env, obj)->read(utf_script);
    }
    catch (const std::exception &e)
    {
        env->ThrowNew(env->FindClass("it/cnr/istc/pst/oratio/SolverException"), e.what());
    }
    if (is_copy == JNI_TRUE)
        env->ReleaseStringUTFChars(script, utf_script);
}

JNIEXPORT void JNICALL Java_it_cnr_istc_pst_oratio_Solver_read___3Ljava_lang_String_2(JNIEnv *env, jobject obj, jobjectArray files)
{
    std::vector<std::string> c_files;
    for (jsize i = 0; i < env->GetArrayLength(files); ++i)
    {
        jboolean is_copy;
        jstring c_file = reinterpret_cast<jstring>(env->GetObjectArrayElement(files, i));
        const char *utf_file = env->GetStringUTFChars(c_file, &is_copy);
        c_files.push_back(utf_file);
        if (is_copy == JNI_TRUE)
            env->ReleaseStringUTFChars(c_file, utf_file);
    }
    try
    {
        get_solver(env, obj)->read(c_files);
    }
    catch (const std::exception &e)
    {
        env->ThrowNew(env->FindClass("it/cnr/istc/pst/oratio/SolverException"), e.what());
    }
}

JNIEXPORT void JNICALL Java_it_cnr_istc_pst_oratio_Solver_solve(JNIEnv *env, jobject obj)
{
    try
    {
        get_solver(env, obj)->solve();
    }
    catch (const std::exception &e)
    {
        env->ThrowNew(env->FindClass("it/cnr/istc/pst/oratio/SolverException"), e.what());
    }
}
