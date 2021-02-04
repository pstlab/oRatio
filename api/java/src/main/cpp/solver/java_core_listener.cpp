#include "java_core_listener.h"
#include "type.h"

namespace ratio
{

    java_core_listener::java_core_listener(core &cr, JNIEnv *env, jobject obj) : core_listener(cr), env(env), obj(obj), solver_cls(env->GetObjectClass(obj)),
                                                                                 log_mthd_id(env->GetMethodID(solver_cls, "fireLog", "(Ljava/lang/String;)V")),
                                                                                 read0_mthd_id(env->GetMethodID(solver_cls, "fireRead", "(Ljava/lang/String;)V")),
                                                                                 read1_mthd_id(env->GetMethodID(solver_cls, "fireRead", "([Ljava/lang/String;)V")),
                                                                                 state_changed_mthd_id(env->GetMethodID(solver_cls, "fireStateChanged", "()V")),
                                                                                 type_cls(env->FindClass("it/cnr/istc/pst/oratio/Type")),
                                                                                 type_ctr_id(env->GetMethodID(type_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Lit/cnr/istc/pst/oratio/Scope;Ljava/lang/String;)V"))
    {
    }
    java_core_listener::~java_core_listener() {}

    void java_core_listener::log(const std::string &msg)
    {
        // the message..
        jstring c_msg = env->NewStringUTF(msg.c_str());

        env->CallVoidMethod(obj, log_mthd_id, c_msg);
        env->DeleteLocalRef(c_msg);
    }

    void java_core_listener::read(const std::string &script)
    {
        // the script..
        jstring c_script = env->NewStringUTF(script.c_str());

        env->CallVoidMethod(obj, read0_mthd_id, c_script);
        env->DeleteLocalRef(c_script);
    }

    void java_core_listener::read(const std::vector<std::string> &files)
    {
        std::vector<jstring> c_files;
        c_files.reserve(files.size());
        for (const auto &f : files)
            c_files.push_back(env->NewStringUTF(f.c_str()));

        jobjectArray files_array = env->NewObjectArray(static_cast<jsize>(c_files.size()), env->FindClass("java/lang/String"), env->NewStringUTF(""));
        for (size_t i = 0; i < c_files.size(); ++i)
            env->SetObjectArrayElement(files_array, static_cast<jsize>(i), c_files[i]);

        env->CallVoidMethod(obj, read1_mthd_id, c_files);

        for (const auto &f : c_files)
            env->DeleteLocalRef(f);
    }

    void java_core_listener::state_changed()
    {
        all_types.emplace(reinterpret_cast<jlong>(&cr), obj);
        std::queue<type *> q;
        for (const auto &t : cr.get_types())
            if (!t.second->is_primitive())
                q.push(t.second);
        while (!q.empty())
        {
            type &t = *q.front();
            q.pop();

            jlong t_id = reinterpret_cast<jlong>(&t);
            if (!all_types.count(reinterpret_cast<jlong>(&t)))
            { // we have a new type..
                jobject c_scope = all_types.at(reinterpret_cast<jlong>(&t.get_scope()));
                jstring t_name = env->NewStringUTF(t.get_name().c_str());
                jobject c_type = env->NewObject(type_cls, type_ctr_id, obj, c_scope, t_name);
                env->DeleteLocalRef(t_name);

                all_types.emplace(t_id, c_type);
            }

            for (const auto &t : t.get_types())
                q.push(t.second);
        }

        env->CallVoidMethod(obj, state_changed_mthd_id);
    }
} // namespace ratio
