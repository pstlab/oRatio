#include "java_core_listener.h"
#include "predicate.h"
#include "field.h"
#include "constructor.h"
#include "method.h"

namespace ratio
{

    java_core_listener::java_core_listener(core &cr, JNIEnv *env, jobject obj) : core_listener(cr), env(env), slv_obj(env->NewGlobalRef(obj)), solver_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->GetObjectClass(obj)))),
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
                                                                                 type_ctr_id(env->GetMethodID(type_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Lit/cnr/istc/pst/oratio/Scope;Ljava/lang/String;Z)V")),
                                                                                 t_dfn_constructor_mthd_id(env->GetMethodID(type_cls, "defineConstructor", "(Lit/cnr/istc/pst/oratio/Constructor;)V")),
                                                                                 t_dfn_superclass_mthd_id(env->GetMethodID(type_cls, "defineSuperclass", "(Lit/cnr/istc/pst/oratio/Type;)V")),
                                                                                 t_dfn_field_mthd_id(env->GetMethodID(type_cls, "defineField", "(Lit/cnr/istc/pst/oratio/Field;)V")),
                                                                                 t_dfn_method_mthd_id(env->GetMethodID(type_cls, "defineMethod", "(Lit/cnr/istc/pst/oratio/Method;)V")),
                                                                                 t_dfn_type_mthd_id(env->GetMethodID(type_cls, "defineType", "(Lit/cnr/istc/pst/oratio/Type;)V")),
                                                                                 t_dfn_pred_mthd_id(env->GetMethodID(type_cls, "definePredicate", "(Lit/cnr/istc/pst/oratio/Predicate;)V")),
                                                                                 ctr_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Constructor")))),
                                                                                 ctr_ctr_id(env->GetMethodID(ctr_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Lit/cnr/istc/pst/oratio/Scope;[Lit/cnr/istc/pst/oratio/Field;)V")),
                                                                                 mthd_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Method")))),
                                                                                 mthd_ctr_id(env->GetMethodID(mthd_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Lit/cnr/istc/pst/oratio/Scope;Ljava/lang/String;Lit/cnr/istc/pst/oratio/Type;[Lit/cnr/istc/pst/oratio/Field;)V")),
                                                                                 field_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Field")))),
                                                                                 field_ctr_id(env->GetMethodID(field_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Type;Ljava/lang/String;)V")),
                                                                                 predicate_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Predicate")))),
                                                                                 predicate_ctr_id(env->GetMethodID(predicate_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Lit/cnr/istc/pst/oratio/Scope;Ljava/lang/String;[Lit/cnr/istc/pst/oratio/Field;)V")),
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
        env->DeleteGlobalRef(ctr_cls);
        env->DeleteGlobalRef(mthd_cls);
        env->DeleteGlobalRef(field_cls);
        env->DeleteGlobalRef(predicate_cls);
        env->DeleteGlobalRef(item_cls);
        env->DeleteGlobalRef(slv_obj);
    }

    void java_core_listener::log(const std::string &msg)
    {
        // the message..
        jstring c_msg = env->NewStringUTF(msg.c_str());

        env->CallVoidMethod(slv_obj, log_mthd_id, c_msg);
        env->DeleteLocalRef(c_msg);
    }

    void java_core_listener::read(const std::string &script)
    {
        // the script..
        jstring c_script = env->NewStringUTF(script.c_str());

        env->CallVoidMethod(slv_obj, read0_mthd_id, c_script);
        env->DeleteLocalRef(c_script);
    }

    void java_core_listener::read(const std::vector<std::string> &files)
    {
        std::vector<jstring> c_files;
        c_files.reserve(files.size());
        for (const auto &f : files)
            c_files.push_back(env->NewStringUTF(f.c_str()));

        jobjectArray files_array = env->NewObjectArray(static_cast<jsize>(c_files.size()), env->FindClass("java/lang/String"), NULL);
        for (size_t i = 0; i < c_files.size(); ++i)
            env->SetObjectArrayElement(files_array, static_cast<jsize>(i), c_files[i]);

        env->CallVoidMethod(slv_obj, read1_mthd_id, c_files);

        for (const auto &f : c_files)
            env->DeleteLocalRef(f);
    }

    void java_core_listener::state_changed()
    {
        std::unordered_set<type *> new_types;
        std::unordered_set<predicate *> new_predicates;
        std::queue<type *> q;
        for (const auto &t : cr.get_types())
            q.push(t.second);
        while (!q.empty())
        {
            type &t = *q.front();
            q.pop();

            jlong t_id = reinterpret_cast<jlong>(&t);
            if (!all_types.count(t_id))
            { // we have a new type..
                if (!t.is_primitive())
                    new_types.insert(&t);
                new_type(t);
            }

            for (const auto &t : t.get_types())
                q.push(t.second);
        }

        // we revise the types..
        for (const auto &t : new_types)
        {
            revise_type(*t);

            // we add the predicates..
            for (const auto &pred : t->get_predicates())
            {
                new_predicates.insert(pred.second);
                new_predicate(*pred.second);
            }
        }

        // we add the methods..
        for (const auto &mthd : cr.get_methods())
        {
            jstring m_name = env->NewStringUTF(mthd->get_name().c_str());
            jobject rt = mthd->get_return_type() ? all_types.at(reinterpret_cast<jlong>(mthd->get_return_type())) : NULL;
            std::vector<jobject> c_fields;
            c_fields.reserve(mthd->get_fields().size());
            for (const auto &f : mthd->get_fields())
            {
                jstring f_name = env->NewStringUTF(f.first.c_str());
                jobject c_field = env->NewObject(field_cls, field_ctr_id, all_types.at(reinterpret_cast<jlong>(&f.second->get_type())), f_name);
                c_fields.push_back(c_field);
                env->DeleteLocalRef(f_name);
            }

            jobjectArray fields_array = env->NewObjectArray(static_cast<jsize>(c_fields.size()), field_cls, NULL);
            for (size_t i = 0; i < c_fields.size(); ++i)
                env->SetObjectArrayElement(fields_array, static_cast<jsize>(i), c_fields[i]);

            jobject c_method = env->NewObject(mthd_cls, mthd_ctr_id, slv_obj, slv_obj, m_name, rt, fields_array);
            env->CallVoidMethod(slv_obj, s_dfn_method_mthd_id, c_method);

            env->DeleteLocalRef(m_name);
            for (size_t i = 0; i < c_fields.size(); ++i)
                env->DeleteLocalRef(c_fields[i]);
            env->DeleteLocalRef(fields_array);
            env->DeleteLocalRef(c_method);
        }

        // we add the predicates..
        for (const auto &pred : cr.get_predicates())
        {
            jlong p_id = reinterpret_cast<jlong>(pred.second);
            if (!all_types.count(p_id))
            {
                new_predicates.insert(pred.second);
                new_predicate(*pred.second);
            }
        }

        // we revise the predicates..
        for (const auto &pred : new_predicates)
            revise_predicate(*pred);

        env->CallVoidMethod(slv_obj, state_changed_mthd_id);
    }

    void java_core_listener::new_type(const type &t)
    {
        jstring t_name = env->NewStringUTF(t.get_name().c_str());
        jobject c_type;

        if (const core *c = dynamic_cast<const core *>(&t.get_scope()))
        {
            c_type = env->NewGlobalRef(env->NewObject(type_cls, type_ctr_id, slv_obj, slv_obj, t_name, t.is_primitive()));
            env->CallVoidMethod(slv_obj, s_dfn_type_mthd_id, c_type);
        }
        else
        {
            jobject c_scope = all_types.at(reinterpret_cast<jlong>(&t.get_scope()));
            c_type = env->NewGlobalRef(env->NewObject(type_cls, type_ctr_id, slv_obj, c_scope, t_name, t.is_primitive()));
            env->CallVoidMethod(c_scope, t_dfn_type_mthd_id, c_type);
        }
        all_types.emplace(reinterpret_cast<jlong>(&t), c_type);
        env->DeleteLocalRef(t_name);
    }

    void java_core_listener::revise_type(const type &t)
    {
        jobject c_type = all_types.at(reinterpret_cast<jlong>(&t));
        // we add the type fields..
        for (const auto &f : t.get_fields())
        {
            jstring f_name = env->NewStringUTF(f.first.c_str());
            jobject c_field = env->NewObject(field_cls, field_ctr_id, all_types.at(reinterpret_cast<jlong>(&f.second->get_type())), f_name);

            env->CallVoidMethod(c_type, t_dfn_field_mthd_id, c_field);

            env->DeleteLocalRef(c_field);
            env->DeleteLocalRef(f_name);
        }

        // we add the supertypes..
        for (const auto &st : t.get_supertypes())
            env->CallVoidMethod(c_type, t_dfn_superclass_mthd_id, all_types.at(reinterpret_cast<jlong>(st)));

        // we add the constructors..
        for (const auto &ctr : t.get_constructors())
        {
            std::vector<jobject> c_fields;
            c_fields.reserve(ctr->get_fields().size());
            for (const auto &f : ctr->get_fields())
            {
                jstring f_name = env->NewStringUTF(f.first.c_str());
                jobject c_field = env->NewObject(field_cls, field_ctr_id, all_types.at(reinterpret_cast<jlong>(&f.second->get_type())), f_name);
                c_fields.push_back(c_field);
                env->DeleteLocalRef(f_name);
            }

            jobjectArray fields_array = env->NewObjectArray(static_cast<jsize>(c_fields.size()), field_cls, NULL);
            for (size_t i = 0; i < c_fields.size(); ++i)
                env->SetObjectArrayElement(fields_array, static_cast<jsize>(i), c_fields[i]);

            jobject c_constructor = env->NewObject(ctr_cls, ctr_ctr_id, slv_obj, c_type, fields_array);
            env->CallVoidMethod(c_type, t_dfn_constructor_mthd_id, c_constructor);

            for (size_t i = 0; i < c_fields.size(); ++i)
                env->DeleteLocalRef(c_fields[i]);
            env->DeleteLocalRef(fields_array);
            env->DeleteLocalRef(c_constructor);
        }

        // we add the methods..
        for (const auto &mthd : t.get_methods())
        {
            jstring m_name = env->NewStringUTF(mthd->get_name().c_str());
            jobject rt = mthd->get_return_type() ? all_types.at(reinterpret_cast<jlong>(mthd->get_return_type())) : NULL;
            std::vector<jobject> c_fields;
            c_fields.reserve(mthd->get_fields().size());
            for (const auto &f : mthd->get_fields())
            {
                jstring f_name = env->NewStringUTF(f.first.c_str());
                jobject c_field = env->NewObject(field_cls, field_ctr_id, all_types.at(reinterpret_cast<jlong>(&f.second->get_type())), f_name);
                c_fields.push_back(c_field);
                env->DeleteLocalRef(f_name);
            }

            jobjectArray fields_array = env->NewObjectArray(static_cast<jsize>(c_fields.size()), field_cls, NULL);
            for (size_t i = 0; i < c_fields.size(); ++i)
                env->SetObjectArrayElement(fields_array, static_cast<jsize>(i), c_fields[i]);

            jobject c_method = env->NewObject(mthd_cls, mthd_ctr_id, slv_obj, c_type, m_name, rt, fields_array);
            env->CallVoidMethod(c_type, t_dfn_method_mthd_id, c_method);

            env->DeleteLocalRef(m_name);
            for (size_t i = 0; i < c_fields.size(); ++i)
                env->DeleteLocalRef(c_fields[i]);
            env->DeleteLocalRef(fields_array);
            env->DeleteLocalRef(c_method);
        }
    }

    void java_core_listener::new_predicate(const predicate &p)
    {
        jstring p_name = env->NewStringUTF(p.get_name().c_str());

        std::vector<jobject> c_fields;
        c_fields.reserve(p.get_fields().size());
        for (const auto &f : p.get_fields())
        {
            jstring f_name = env->NewStringUTF(f.first.c_str());
            jobject c_field = env->NewObject(field_cls, field_ctr_id, all_types.at(reinterpret_cast<jlong>(&f.second->get_type())), f_name);
            c_fields.push_back(c_field);
            env->DeleteLocalRef(f_name);
        }

        jobjectArray fields_array = env->NewObjectArray(static_cast<jsize>(c_fields.size()), field_cls, NULL);
        for (size_t i = 0; i < c_fields.size(); ++i)
            env->SetObjectArrayElement(fields_array, static_cast<jsize>(i), c_fields[i]);

        jobject c_predicate;
        if (const core *c = dynamic_cast<const core *>(&p.get_scope()))
        {
            c_predicate = env->NewGlobalRef(env->NewObject(predicate_cls, predicate_ctr_id, slv_obj, slv_obj, p_name, fields_array));
            env->CallVoidMethod(slv_obj, s_dfn_pred_mthd_id, c_predicate);
        }
        else
        {
            jobject c_scope = all_types.at(reinterpret_cast<jlong>(&p.get_scope()));
            c_predicate = env->NewGlobalRef(env->NewObject(predicate_cls, predicate_ctr_id, slv_obj, c_scope, p_name, fields_array));
            env->CallVoidMethod(c_scope, t_dfn_pred_mthd_id, c_predicate);
        }
        all_types.emplace(reinterpret_cast<jlong>(&p), c_predicate);

        env->DeleteLocalRef(p_name);
        for (size_t i = 0; i < c_fields.size(); ++i)
            env->DeleteLocalRef(c_fields[i]);
        env->DeleteLocalRef(fields_array);
    }

    void java_core_listener::revise_predicate(const predicate &p)
    {
        jobject c_pred = all_types.at(reinterpret_cast<jlong>(&p));
        for (const auto &sp : p.get_supertypes())
            env->CallVoidMethod(c_pred, t_dfn_superclass_mthd_id, all_types.at(reinterpret_cast<jlong>(sp)));
    }
} // namespace ratio
