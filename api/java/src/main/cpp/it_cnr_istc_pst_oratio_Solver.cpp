#include "it_cnr_istc_pst_oratio_Solver.h"
#include "solver.h"
#ifdef BUILD_GUI
#include "solver_socket_listener.h"
#endif
#include <sstream>

using namespace ratio;

inline solver *get_solver(JNIEnv *env, jobject obj) { return reinterpret_cast<solver *>(env->GetLongField(obj, env->GetFieldID(env->GetObjectClass(obj), "native_handle", "J"))); }

JNIEXPORT jlong JNICALL Java_it_cnr_istc_oratio_Solver_new_1instance(JNIEnv *env, jobject obj)
{
  solver *s = new solver();
#ifdef BUILD_GUI
  solver_socket_listener *l = new solver_socket_listener(*s, HOST, PORT);
  env->SetLongField(obj, env->GetFieldID(env->GetObjectClass(obj), "native_listener_handle", "J"), reinterpret_cast<jlong>(l));
#endif

  s->init();
  return reinterpret_cast<jlong>(s);
}

JNIEXPORT void JNICALL Java_it_cnr_istc_oratio_Solver_dispose(JNIEnv *env, jobject obj)
{
#ifdef BUILD_GUI
  delete reinterpret_cast<solver_socket_listener *>(env->GetLongField(obj, env->GetFieldID(env->GetObjectClass(obj), "native_listener_handle", "J")));
  env->SetLongField(obj, env->GetFieldID(env->GetObjectClass(obj), "native_listener_handle", "J"), 0);
#endif

  delete get_solver(env, obj);
  env->SetLongField(obj, env->GetFieldID(env->GetObjectClass(obj), "native_handle", "J"), 0);
}

JNIEXPORT void JNICALL Java_it_cnr_istc_oratio_Solver_read__Ljava_lang_String_2(JNIEnv *env, jobject obj, jstring script) { get_solver(env, obj)->read(env->GetStringUTFChars(script, (jboolean) false)); }

JNIEXPORT void JNICALL Java_it_cnr_istc_oratio_Solver_read___3Ljava_lang_String_2(JNIEnv *env, jobject obj, jobjectArray files)
{
  std::vector<std::string> c_files;
  for (jsize i = 0; i < env->GetArrayLength(files); ++i)
    c_files.push_back(env->GetStringUTFChars(reinterpret_cast<jstring>(env->GetObjectArrayElement(files, i)), (jboolean) false));
  get_solver(env, obj)->read(c_files);
}

JNIEXPORT void JNICALL Java_it_cnr_istc_oratio_Solver_solve(JNIEnv *env, jobject obj) { get_solver(env, obj)->solve(); }

JNIEXPORT jstring JNICALL Java_it_cnr_istc_oratio_Solver_getState(JNIEnv *env, jobject obj)
{
  std::stringstream ss;
  ss << get_solver(env, obj);
  return env->NewStringUTF(ss.str().c_str());
}
