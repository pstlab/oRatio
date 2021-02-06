#include "java_core_listener.h"
#include "type.h"

namespace ratio
{

    java_core_listener::java_core_listener(core &cr, JNIEnv *env, jobject obj) : core_listener(cr), env(env), obj(env->NewGlobalRef(obj)), solver_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->GetObjectClass(obj)))),
                                                                                 log_mthd_id(env->GetMethodID(solver_cls, "fireLog", "(Ljava/lang/String;)V")),
                                                                                 read0_mthd_id(env->GetMethodID(solver_cls, "fireRead", "(Ljava/lang/String;)V")),
                                                                                 read1_mthd_id(env->GetMethodID(solver_cls, "fireRead", "([Ljava/lang/String;)V")),
                                                                                 state_changed_mthd_id(env->GetMethodID(solver_cls, "fireStateChanged", "()V")),
                                                                                 s_dfn_field_mthd_id(env->GetMethodID(solver_cls, "defineField", "(Lit/cnr/istc/pst/oratio/Field;)V")),
                                                                                 s_dfn_method_mthd_id(env->GetMethodID(solver_cls, "defineMethod", "(Lit/cnr/istc/pst/oratio/Method;)V")),
                                                                                 s_dfn_type_mthd_id(env->GetMethodID(solver_cls, "defineType", "(Lit/cnr/istc/pst/oratio/Type;)V")),
                                                                                 s_dfn_pred_mthd_id(env->GetMethodID(solver_cls, "definePredicate", "(Lit/cnr/istc/pst/oratio/Predicate;)V")),
                                                                                 s_set_mthd_id(env->GetMethodID(solver_cls, "set", "(Ljava/lang/String;Lit/cnr/istc/pst/oratio/Item;)V")),
                                                                                 type_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Type")))),
                                                                                 type_ctr_id(env->GetMethodID(type_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Lit/cnr/istc/pst/oratio/Scope;Ljava/lang/String;)V")),
                                                                                 t_dfn_constructor_mthd_id(env->GetMethodID(type_cls, "defineConstructor", "(Lit/cnr/istc/pst/oratio/Constructor;)V")),
                                                                                 t_dfn_field_mthd_id(env->GetMethodID(type_cls, "defineField", "(Lit/cnr/istc/pst/oratio/Field;)V")),
                                                                                 t_dfn_method_mthd_id(env->GetMethodID(type_cls, "defineMethod", "(Lit/cnr/istc/pst/oratio/Method;)V")),
                                                                                 t_dfn_type_mthd_id(env->GetMethodID(type_cls, "defineType", "(Lit/cnr/istc/pst/oratio/Type;)V")),
                                                                                 t_dfn_pred_mthd_id(env->GetMethodID(type_cls, "definePredicate", "(Lit/cnr/istc/pst/oratio/Predicate;)V")),
                                                                                 item_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Item")))),
                                                                                 item_ctr_id(env->GetMethodID(item_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Lit/cnr/istc/pst/oratio/Type;)V")),
                                                                                 i_set_mthd_id(env->GetMethodID(item_cls, "set", "(Ljava/lang/String;Lit/cnr/istc/pst/oratio/Item;)V"))
    {
    }
    java_core_listener::~java_core_listener()
    {
        for (const auto &t : all_items)
            env->DeleteGlobalRef(t.second);
        for (const auto &t : all_types)
            env->DeleteGlobalRef(t.second);

        env->DeleteGlobalRef(solver_cls);
        env->DeleteGlobalRef(type_cls);
        env->DeleteGlobalRef(item_cls);
        env->DeleteGlobalRef(obj);
    }

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
                jstring t_name = env->NewStringUTF(t.get_name().c_str());
                jobject c_type;

                if (const core *c = dynamic_cast<const core *>(&t.get_scope()))
                {
                    c_type = env->NewGlobalRef(env->NewObject(type_cls, type_ctr_id, obj, obj, t_name));
                    env->CallVoidMethod(obj, s_dfn_type_mthd_id, c_type);
                }
                else
                {
                    jobject c_scope = all_types.at(reinterpret_cast<jlong>(&t.get_scope()));
                    c_type = env->NewGlobalRef(env->NewObject(type_cls, type_ctr_id, obj, c_scope, t_name));
                    env->CallVoidMethod(c_scope, t_dfn_type_mthd_id, c_type);
                }
                all_types.emplace(t_id, c_type);
            }

            for (const auto &t : t.get_types())
                q.push(t.second);
        }

        env->CallVoidMethod(obj, state_changed_mthd_id);
    }
} // namespace ratio
