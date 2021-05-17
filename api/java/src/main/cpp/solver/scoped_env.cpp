#include "scoped_env.h"
#include <thread>
#include <cassert>

namespace ratio
{

    scoped_env::scoped_env(JNIEnv *env)
    {
        jint res = env->GetJavaVM(&jvm);
        assert(res == JNI_OK);
    }

    scoped_env::~scoped_env() {}

    JNIEnv *scoped_env::get_env()
    {
        auto get_env_result = jvm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_8);
        if (get_env_result == JNI_EDETACHED)
            if (jvm->AttachCurrentThread(reinterpret_cast<void **>(&env), NULL) == JNI_OK)
                thread_local detach_on_exit tmp(jvm);
            else
                return nullptr;
        else if (get_env_result == JNI_EVERSION)
            return nullptr;
        return env;
    }
} // namespace ratio
