#include "java_executor_listener.h"
#include "atom.h"

namespace ratio
{

    java_executor_listener::java_executor_listener(executor &e, JNIEnv *env, jobject obj) : scoped_env(env), executor_listener(e), exec_obj(env->NewGlobalRef(obj)), exec_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->GetObjectClass(obj)))),
                                                                                            tick_mthd_id(env->GetMethodID(exec_cls, "fireTick", "(JJ)V")),
                                                                                            starting_mthd_id(env->GetMethodID(exec_cls, "fireStartingAtoms", "([J)V")),
                                                                                            start_mthd_id(env->GetMethodID(exec_cls, "fireStartAtoms", "([J)V")),
                                                                                            ending_mthd_id(env->GetMethodID(exec_cls, "fireEndingAtoms", "([J)V")),
                                                                                            end_mthd_id(env->GetMethodID(exec_cls, "fireEndAtoms", "([J)V"))
    {
    }
    java_executor_listener::~java_executor_listener()
    {
        const auto &env = get_env();
        env->DeleteGlobalRef(exec_obj);
        env->DeleteGlobalRef(exec_cls);
    }

    void java_executor_listener::tick(const smt::rational time)
    {
        jlong time_num = time.numerator(), time_den = time.denominator();

        get_env()->CallVoidMethod(exec_obj, tick_mthd_id, time_num, time_den);
    }

    void java_executor_listener::starting(const std::unordered_set<atom *> &atoms) { call_method(starting_mthd_id, atoms); }
    void java_executor_listener::start(const std::unordered_set<atom *> &atoms) { call_method(start_mthd_id, atoms); }

    void java_executor_listener::ending(const std::unordered_set<atom *> &atoms) { call_method(ending_mthd_id, atoms); }
    void java_executor_listener::end(const std::unordered_set<atom *> &atoms) { call_method(end_mthd_id, atoms); }

    void java_executor_listener::call_method(const jmethodID &mthd_id, const std::unordered_set<atom *> &atoms)
    {
        const auto &env = get_env();
        jlongArray atms_array = env->NewLongArray(static_cast<jsize>(atoms.size()));
        std::vector<jlong> c_atms;
        for (const auto &atm : atoms)
            c_atms.push_back(static_cast<jlong>(atm->get_sigma()));
        env->SetLongArrayRegion(atms_array, 0, static_cast<jsize>(c_atms.size()), c_atms.data());

        env->CallVoidMethod(exec_obj, mthd_id, atms_array);

        env->DeleteLocalRef(atms_array);
    }
} // namespace ratio
