#include "java_executor_listener.h"

namespace ratio
{

    java_executor_listener::java_executor_listener(executor &e, JNIEnv *env, jobject obj) : executor_listener(e), env(env), exec_obj(env->NewGlobalRef(obj)), exec_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->GetObjectClass(obj)))),
                                                                                            tick_mthd_id(env->GetMethodID(exec_cls, "fireTick", "(JJ)V")),
                                                                                            starting_mthd_id(env->GetMethodID(exec_cls, "fireStartingAtoms", "([J)V")),
                                                                                            ending_mthd_id(env->GetMethodID(exec_cls, "fireEndingAtoms", "([J)V"))
    {
    }
    java_executor_listener::~java_executor_listener()
    {
        env->DeleteGlobalRef(exec_obj);
        env->DeleteGlobalRef(exec_cls);
    }

    void java_executor_listener::tick(const smt::rational time)
    {
        jlong time_num = time.numerator(), time_den = time.denominator();

        env->CallVoidMethod(exec_obj, tick_mthd_id, time_num, time_den);
    }
    void java_executor_listener::starting(const std::set<atom *> &atoms)
    {
        jlongArray atms_array = env->NewLongArray(static_cast<jsize>(atoms.size()));
        std::vector<jlong> c_atms;
        for (const auto &atm : atoms)
            c_atms.push_back(reinterpret_cast<jlong>(atm));
        env->SetLongArrayRegion(atms_array, 0, static_cast<jsize>(c_atms.size()), c_atms.data());

        env->CallVoidMethod(exec_obj, starting_mthd_id, atms_array);

        env->DeleteLocalRef(atms_array);
    }
    void java_executor_listener::ending(const std::set<atom *> &atoms)
    {
        jlongArray atms_array = env->NewLongArray(static_cast<jsize>(atoms.size()));
        std::vector<jlong> c_atms;
        for (const auto &atm : atoms)
            c_atms.push_back(reinterpret_cast<jlong>(atm));
        env->SetLongArrayRegion(atms_array, 0, static_cast<jsize>(c_atms.size()), c_atms.data());

        env->CallVoidMethod(exec_obj, ending_mthd_id, atms_array);

        env->DeleteLocalRef(atms_array);
    }
} // namespace ratio
