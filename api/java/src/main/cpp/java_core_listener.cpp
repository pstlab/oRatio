#include "java_core_listener.h"
#include "predicate.h"
#include "field.h"
#include "constructor.h"
#include "method.h"
#include "atom.h"

namespace ratio
{

    java_core_listener::java_core_listener(core &cr, JNIEnv *env, jobject obj) : scoped_env(env), core_listener(cr), slv_obj(env->NewGlobalRef(obj)), solver_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->GetObjectClass(obj)))),
                                                                                 log_mthd_id(env->GetMethodID(solver_cls, "fireLog", "(Ljava/lang/String;)V")),
                                                                                 read0_mthd_id(env->GetMethodID(solver_cls, "fireRead", "(Ljava/lang/String;)V")),
                                                                                 read1_mthd_id(env->GetMethodID(solver_cls, "fireRead", "([Ljava/lang/String;)V")),
                                                                                 state_changed_mthd_id(env->GetMethodID(solver_cls, "fireStateChanged", "()V")),
                                                                                 s_dfn_field_mthd_id(env->GetMethodID(solver_cls, "defineField", "(Lit/cnr/istc/pst/oratio/Field;)V")),
                                                                                 s_dfn_method_mthd_id(env->GetMethodID(solver_cls, "defineMethod", "(Lit/cnr/istc/pst/oratio/Method;)V")),
                                                                                 s_dfn_type_mthd_id(env->GetMethodID(solver_cls, "defineType", "(Lit/cnr/istc/pst/oratio/Type;)V")),
                                                                                 s_dfn_pred_mthd_id(env->GetMethodID(solver_cls, "definePredicate", "(Lit/cnr/istc/pst/oratio/Predicate;)V")),
                                                                                 s_set_mthd_id(env->GetMethodID(solver_cls, "set", "(Ljava/lang/String;Lit/cnr/istc/pst/oratio/Item;)V")),
                                                                                 s_strtd_slvng_mthd_id(env->GetMethodID(solver_cls, "fireStartedSolving", "()V")),
                                                                                 s_sol_found_mthd_id(env->GetMethodID(solver_cls, "fireSolutionFound", "()V")),
                                                                                 s_inc_prblm_mthd_id(env->GetMethodID(solver_cls, "fireInconsistentProblem", "()V")),
                                                                                 type_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Type")))),
                                                                                 type_ctr_id(env->GetMethodID(type_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Lit/cnr/istc/pst/oratio/Scope;Ljava/lang/String;Z)V")),
                                                                                 t_dfn_constructor_mthd_id(env->GetMethodID(type_cls, "defineConstructor", "(Lit/cnr/istc/pst/oratio/Constructor;)V")),
                                                                                 t_dfn_superclass_mthd_id(env->GetMethodID(type_cls, "defineSuperclass", "(Lit/cnr/istc/pst/oratio/Type;)V")),
                                                                                 t_dfn_field_mthd_id(env->GetMethodID(type_cls, "defineField", "(Lit/cnr/istc/pst/oratio/Field;)V")),
                                                                                 t_dfn_method_mthd_id(env->GetMethodID(type_cls, "defineMethod", "(Lit/cnr/istc/pst/oratio/Method;)V")),
                                                                                 t_dfn_type_mthd_id(env->GetMethodID(type_cls, "defineType", "(Lit/cnr/istc/pst/oratio/Type;)V")),
                                                                                 t_dfn_pred_mthd_id(env->GetMethodID(type_cls, "definePredicate", "(Lit/cnr/istc/pst/oratio/Predicate;)V")),
                                                                                 t_new_instnc(env->GetMethodID(type_cls, "newInstance", "(Lit/cnr/istc/pst/oratio/Item;)V")),
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
                                                                                 i_set_name_mthd_id(env->GetMethodID(item_cls, "setName", "(Ljava/lang/String;)V")),
                                                                                 i_set_mthd_id(env->GetMethodID(item_cls, "set", "(Ljava/lang/String;Lit/cnr/istc/pst/oratio/Item;)V")),
                                                                                 bool_item_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Item$BoolItem")))),
                                                                                 bool_item_ctr_id(env->GetMethodID(bool_item_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Ljava/lang/String;B)V")),
                                                                                 bool_item_set_mthd_id(env->GetMethodID(bool_item_cls, "setValue", "(B)V")),
                                                                                 rat_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Rational")))),
                                                                                 rat_ctr_id(env->GetMethodID(rat_cls, "<init>", "(JJ)V")),
                                                                                 inf_rat_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/InfRational")))),
                                                                                 inf_rat_ctr_id(env->GetMethodID(inf_rat_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Rational;Lit/cnr/istc/pst/oratio/Rational;)V")),
                                                                                 arith_item_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Item$ArithItem")))),
                                                                                 arith_item_ctr_id(env->GetMethodID(arith_item_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Lit/cnr/istc/pst/oratio/Type;Ljava/lang/String;Lit/cnr/istc/pst/oratio/InfRational;Lit/cnr/istc/pst/oratio/InfRational;Lit/cnr/istc/pst/oratio/InfRational;)V")),
                                                                                 arith_item_set_lb_mthd_id(env->GetMethodID(arith_item_cls, "setLb", "(JJJJ)V")),
                                                                                 arith_item_set_ub_mthd_id(env->GetMethodID(arith_item_cls, "setUb", "(JJJJ)V")),
                                                                                 arith_item_set_val_mthd_id(env->GetMethodID(arith_item_cls, "setVal", "(JJJJ)V")),
                                                                                 enum_item_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Item$EnumItem")))),
                                                                                 enum_item_ctr_id(env->GetMethodID(enum_item_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Lit/cnr/istc/pst/oratio/Type;Ljava/lang/String;[Lit/cnr/istc/pst/oratio/Item;)V")),
                                                                                 enum_item_set_mthd_id(env->GetMethodID(enum_item_cls, "setVals", "([Lit/cnr/istc/pst/oratio/Item;)V")),
                                                                                 string_item_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Item$StringItem")))),
                                                                                 string_item_ctr_id(env->GetMethodID(string_item_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Ljava/lang/String;)V")),
                                                                                 string_item_set_mthd_id(env->GetMethodID(string_item_cls, "setValue", "(Ljava/lang/String;)V")),
                                                                                 atm_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->FindClass("it/cnr/istc/pst/oratio/Atom")))),
                                                                                 atm_ctr_id(env->GetMethodID(atm_cls, "<init>", "(Lit/cnr/istc/pst/oratio/Solver;Lit/cnr/istc/pst/oratio/Predicate;JB)V")),
                                                                                 atm_set_state_mthd_id(env->GetMethodID(atm_cls, "setState", "(B)V"))
    {
    }
    java_core_listener::~java_core_listener()
    {
        const auto &env = get_env();
        for (const auto &t : all_items)
            env->DeleteGlobalRef(t.second);
        for (const auto &t : all_types)
            env->DeleteGlobalRef(t.second);

        env->DeleteGlobalRef(type_cls);
        env->DeleteGlobalRef(ctr_cls);
        env->DeleteGlobalRef(mthd_cls);
        env->DeleteGlobalRef(field_cls);
        env->DeleteGlobalRef(predicate_cls);
        env->DeleteGlobalRef(item_cls);
        env->DeleteGlobalRef(slv_obj);
        env->DeleteGlobalRef(solver_cls);
    }

    void java_core_listener::log(const std::string &msg)
    {
        const auto &env = get_env();
        // the message..
        jstring c_msg = env->NewStringUTF(msg.c_str());

        env->CallVoidMethod(slv_obj, log_mthd_id, c_msg);
        env->DeleteLocalRef(c_msg);
    }

    void java_core_listener::read(const std::string &script)
    {
        const auto &env = get_env();
        // the script..
        jstring c_script = env->NewStringUTF(script.c_str());

        env->CallVoidMethod(slv_obj, read0_mthd_id, c_script);
        env->DeleteLocalRef(c_script);
    }

    void java_core_listener::read(const std::vector<std::string> &files)
    {
        const auto &env = get_env();
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
        const auto &env = get_env();
        std::unordered_set<type *> new_types;
        std::unordered_set<predicate *> new_predicates;
        std::queue<type *> q;
        for (const auto &[tp_name, tp] : cr.get_types())
            q.push(tp);
        while (!q.empty())
        {
            type &t = *q.front();
            q.pop();

            if (const auto &t_it = all_types.find(&t); t_it == all_types.cend())
            { // we have a new type..
                if (!t.is_primitive())
                    new_types.insert(&t);
                new_type(env, t);
            }

            for (const auto &[tp_name, tp] : t.get_types())
                q.push(tp);
        }

        // we revise the types..
        for (const auto &t : new_types)
        {
            revise_type(env, *t);

            // we add the predicates..
            for (const auto &[pred_name, pred] : t->get_predicates())
            {
                new_predicates.insert(pred);
                new_predicate(env, *pred);
            }
        }

        // we add the methods..
        for (const auto &mthd : cr.get_methods())
        {
            jstring m_name = env->NewStringUTF(mthd->get_name().c_str());
            jobject rt = mthd->get_return_type() ? all_types.at(mthd->get_return_type()) : NULL;
            jobjectArray fields_array = new_fields_array(env, mthd->get_args());

            jobject c_method = env->NewObject(mthd_cls, mthd_ctr_id, slv_obj, slv_obj, m_name, rt, fields_array);
            env->CallVoidMethod(slv_obj, s_dfn_method_mthd_id, c_method);

            env->DeleteLocalRef(c_method);
            env->DeleteLocalRef(fields_array);
            env->DeleteLocalRef(m_name);
        }

        // we add the predicates..
        for (const auto &[pred_name, pred] : cr.get_predicates())
            if (const auto &p_it = all_types.find(pred); p_it == all_types.cend())
            {
                new_predicates.insert(pred);
                new_predicate(env, *pred);
            }

        // we revise the predicates..
        for (const auto &pred : new_predicates)
            revise_predicate(env, *pred);

        // we add items and atoms..
        std::unordered_set<item *> c_items;
        for (const auto &[pred_name, pred] : cr.get_predicates())
            for (const auto &a : pred->get_instances())
            {
                atom &atm = static_cast<atom &>(*a);
                c_items.insert(&atm);
                const auto &i_it = all_items.find(&atm);
                if (i_it == all_items.cend())
                    new_atom(env, atm);
            }
        for (const auto &[tp_name, tp] : cr.get_types())
            if (!tp->is_primitive())
                q.push(tp);
        while (!q.empty())
        {
            for (const auto &i : q.front()->get_instances())
            {
                item &itm = *i;
                c_items.insert(&itm);
                if (const auto &i_it = all_items.find(&itm); i_it == all_items.cend())
                    new_item(env, itm);
            }
            for (const auto &[pred_name, pred] : q.front()->get_predicates())
                for (const auto &a : pred->get_instances())
                {
                    atom &atm = static_cast<atom &>(*a);
                    c_items.insert(&atm);
                    if (const auto &a_it = all_items.find(&atm); a_it == all_items.cend())
                        new_atom(env, atm);
                }
            for (const auto &[tp_name, tp] : q.front()->get_types())
                q.push(tp);
            q.pop();
        }

        for (const auto &[itm, j_itm] : std::unordered_map<const item *, jobject>(all_items))
        {
            for (const auto &[xpr_name, xpr] : itm->get_exprs())
                set(env, j_itm, i_set_mthd_id, xpr_name, *xpr);

            if (const atom *a = dynamic_cast<const atom *>(itm))
                env->CallVoidMethod(j_itm, atm_set_state_mthd_id, cr.get_sat_core().value(a->get_sigma()));
        }

        for (const auto &[xpr_name, xpr] : cr.get_exprs())
            if (jobject obj = set(env, slv_obj, s_set_mthd_id, xpr_name, *xpr))
            {
                jstring c_name = env->NewStringUTF(cr.guess_name(*xpr).c_str());
                env->CallVoidMethod(obj, i_set_name_mthd_id, c_name);
                env->DeleteLocalRef(c_name);
            }

        env->CallVoidMethod(slv_obj, state_changed_mthd_id);
    }

    void java_core_listener::started_solving() { get_env()->CallVoidMethod(slv_obj, s_strtd_slvng_mthd_id); }
    void java_core_listener::solution_found() { get_env()->CallVoidMethod(slv_obj, s_sol_found_mthd_id); }
    void java_core_listener::inconsistent_problem() { get_env()->CallVoidMethod(slv_obj, s_inc_prblm_mthd_id); }

    void java_core_listener::new_type(JNIEnv *env, const type &t)
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
            jobject c_scope = all_types.at(static_cast<type *>(&t.get_scope()));
            c_type = env->NewGlobalRef(env->NewObject(type_cls, type_ctr_id, slv_obj, c_scope, t_name, t.is_primitive()));
            env->CallVoidMethod(c_scope, t_dfn_type_mthd_id, c_type);
        }
        all_types.emplace(&t, c_type);
        env->DeleteLocalRef(t_name);
    }

    void java_core_listener::revise_type(JNIEnv *env, const type &t)
    {
        jobject c_type = all_types.at(&t);
        // we add the type fields..
        for (const auto &[f_name, f] : t.get_fields())
        {
            jstring j_name = env->NewStringUTF(f_name.c_str());
            jobject c_field = env->NewObject(field_cls, field_ctr_id, all_types.at(&f->get_type()), j_name);

            env->CallVoidMethod(c_type, t_dfn_field_mthd_id, c_field);

            env->DeleteLocalRef(c_field);
            env->DeleteLocalRef(j_name);
        }

        // we add the supertypes..
        for (const auto &st : t.get_supertypes())
            env->CallVoidMethod(c_type, t_dfn_superclass_mthd_id, all_types.at(st));

        // we add the constructors..
        for (const auto &ctr : t.get_constructors())
        {
            jobjectArray fields_array = new_fields_array(env, ctr->get_args());

            jobject c_constructor = env->NewObject(ctr_cls, ctr_ctr_id, slv_obj, c_type, fields_array);
            env->CallVoidMethod(c_type, t_dfn_constructor_mthd_id, c_constructor);

            env->DeleteLocalRef(fields_array);
            env->DeleteLocalRef(c_constructor);
        }

        // we add the methods..
        for (const auto &mthd : t.get_methods())
        {
            jstring m_name = env->NewStringUTF(mthd->get_name().c_str());
            jobject rt = mthd->get_return_type() ? all_types.at(mthd->get_return_type()) : NULL;
            jobjectArray fields_array = new_fields_array(env, mthd->get_args());

            jobject c_method = env->NewObject(mthd_cls, mthd_ctr_id, slv_obj, c_type, m_name, rt, fields_array);
            env->CallVoidMethod(c_type, t_dfn_method_mthd_id, c_method);

            env->DeleteLocalRef(c_method);
            env->DeleteLocalRef(fields_array);
            env->DeleteLocalRef(m_name);
        }
    }

    void java_core_listener::new_predicate(JNIEnv *env, const predicate &p)
    {
        jstring p_name = env->NewStringUTF(p.get_name().c_str());
        jobjectArray fields_array = new_fields_array(env, p.get_args());

        jobject c_predicate;
        if (const core *c = dynamic_cast<const core *>(&p.get_scope()))
        {
            c_predicate = env->NewGlobalRef(env->NewObject(predicate_cls, predicate_ctr_id, slv_obj, slv_obj, p_name, fields_array));
            env->CallVoidMethod(slv_obj, s_dfn_pred_mthd_id, c_predicate);
        }
        else
        {
            jobject c_scope = all_types.at(static_cast<type *>(&p.get_scope()));
            c_predicate = env->NewGlobalRef(env->NewObject(predicate_cls, predicate_ctr_id, slv_obj, c_scope, p_name, fields_array));
            env->CallVoidMethod(c_scope, t_dfn_pred_mthd_id, c_predicate);
        }
        all_types.emplace(&p, c_predicate);

        env->DeleteLocalRef(fields_array);
        env->DeleteLocalRef(p_name);
    }

    void java_core_listener::revise_predicate(JNIEnv *env, const predicate &p)
    {
        jobject c_pred = all_types.at(&p);
        for (const auto &sp : p.get_supertypes())
            env->CallVoidMethod(c_pred, t_dfn_superclass_mthd_id, all_types.at(sp));
    }

    void java_core_listener::new_item(JNIEnv *env, const item &itm)
    {
        jobject c_type = all_types.at(&itm.get_type());

        jobject c_item = env->NewGlobalRef(env->NewObject(item_cls, item_ctr_id, slv_obj, c_type));
        env->CallVoidMethod(c_type, t_new_instnc, c_item);
        all_items.emplace(&itm, c_item);

        jstring c_name = env->NewStringUTF(cr.guess_name(itm).c_str());
        env->CallVoidMethod(c_item, i_set_name_mthd_id, c_name);
        env->DeleteLocalRef(c_name);
    }

    void java_core_listener::new_atom(JNIEnv *env, const atom &atm)
    {
        jobject c_pred = all_types.at(&atm.get_type());

        jobject c_atom = env->NewGlobalRef(env->NewObject(atm_cls, atm_ctr_id, slv_obj, c_pred, atm.get_sigma(), cr.get_sat_core().value(atm.get_sigma())));
        env->CallVoidMethod(c_pred, t_new_instnc, c_atom);
        all_items.emplace(&atm, c_atom);
    }

    jobject java_core_listener::new_field(JNIEnv *env, const std::string &name, const type &tp)
    {
        jstring f_name = env->NewStringUTF(name.c_str());
        jobject c_field = env->NewObject(field_cls, field_ctr_id, all_types.at(&tp), f_name);
        env->DeleteLocalRef(f_name);
        return c_field;
    }

    jobjectArray java_core_listener::new_fields_array(JNIEnv *env, const std::vector<const field *> &args)
    {
        std::vector<jobject> c_fields;
        c_fields.reserve(args.size());
        for (const auto &f : args)
            c_fields.push_back(new_field(env, f->get_name(), f->get_type()));

        jobjectArray fields_array = env->NewObjectArray(static_cast<jsize>(c_fields.size()), field_cls, NULL);
        for (size_t i = 0; i < c_fields.size(); ++i)
            env->SetObjectArrayElement(fields_array, static_cast<jsize>(i), c_fields[i]);

        for (size_t i = 0; i < c_fields.size(); ++i)
            env->DeleteLocalRef(c_fields[i]);

        return fields_array;
    }

    jobject java_core_listener::set(JNIEnv *env, jobject c_obj, jmethodID mthd_id, const std::string &name, const item &itm)
    {
        const auto &i_it = all_items.find(&itm);
        jobject new_obj = nullptr;
        jstring i_name = env->NewStringUTF(name.c_str());
        if (const bool_item *bi = dynamic_cast<const bool_item *>(&itm))
        { // the expression represents a primitive bool type..
            const auto &c_val = cr.get_sat_core().value(bi->l);
            if (i_it == all_items.cend())
            { // we create a new boolean..
                jstring lit_s = env->NewStringUTF(((sign(bi->l) ? "b" : "!b") + std::to_string(variable(bi->l))).c_str());
                new_obj = env->NewGlobalRef(env->NewObject(bool_item_cls, bool_item_ctr_id, slv_obj, lit_s, c_val));
                env->CallVoidMethod(c_obj, mthd_id, i_name, new_obj);
                all_items.emplace(bi, new_obj);
                env->DeleteLocalRef(lit_s);
            }
            else
            { // we update the value..
                env->CallVoidMethod(i_it->second, bool_item_set_mthd_id, c_val);
                env->CallVoidMethod(c_obj, mthd_id, i_name, i_it->second);
            }
        }
        else if (const arith_item *ai = dynamic_cast<const arith_item *>(&itm))
        { // the expression represents a primitive arithmetic type..
            const auto lb = cr.get_lra_theory().lb(ai->l);
            const auto ub = cr.get_lra_theory().ub(ai->l);
            const auto val = cr.get_lra_theory().value(ai->l);
            jlong lb_rat_num = lb.get_rational().numerator(), lb_rat_den = lb.get_rational().denominator(), lb_inf_num = lb.get_infinitesimal().numerator(), lb_inf_den = lb.get_infinitesimal().denominator();
            jlong ub_rat_num = ub.get_rational().numerator(), ub_rat_den = ub.get_rational().denominator(), ub_inf_num = ub.get_infinitesimal().numerator(), ub_inf_den = ub.get_infinitesimal().denominator();
            jlong val_rat_num = val.get_rational().numerator(), val_rat_den = val.get_rational().denominator(), val_inf_num = val.get_infinitesimal().numerator(), val_inf_den = val.get_infinitesimal().denominator();

            if (i_it == all_items.cend())
            { // we create a new arith..
                jobject c_type = all_types.at(&itm.get_type());
                jstring lit_s = env->NewStringUTF(to_string(ai->l).c_str());
                jobject c_lb = env->NewObject(inf_rat_cls, inf_rat_ctr_id, env->NewObject(rat_cls, rat_ctr_id, lb_rat_num, lb_rat_den), env->NewObject(rat_cls, rat_ctr_id, lb_inf_num, lb_inf_den));
                jobject c_ub = env->NewObject(inf_rat_cls, inf_rat_ctr_id, env->NewObject(rat_cls, rat_ctr_id, ub_rat_num, ub_rat_den), env->NewObject(rat_cls, rat_ctr_id, ub_inf_num, ub_inf_den));
                jobject c_val = env->NewObject(inf_rat_cls, inf_rat_ctr_id, env->NewObject(rat_cls, rat_ctr_id, val_rat_num, val_rat_den), env->NewObject(rat_cls, rat_ctr_id, val_inf_num, val_inf_den));
                new_obj = env->NewGlobalRef(env->NewObject(arith_item_cls, arith_item_ctr_id, slv_obj, c_type, lit_s, c_lb, c_ub, c_val));
                env->CallVoidMethod(c_obj, mthd_id, i_name, new_obj);
                all_items.emplace(ai, new_obj);
                env->DeleteLocalRef(c_val);
                env->DeleteLocalRef(c_ub);
                env->DeleteLocalRef(c_lb);
                env->DeleteLocalRef(lit_s);
            }
            else
            { // we update the value..
                env->CallVoidMethod(i_it->second, arith_item_set_lb_mthd_id, lb_rat_num, lb_rat_den, lb_inf_num, lb_inf_den);
                env->CallVoidMethod(i_it->second, arith_item_set_ub_mthd_id, ub_rat_num, ub_rat_den, ub_inf_num, ub_inf_den);
                env->CallVoidMethod(i_it->second, arith_item_set_val_mthd_id, val_rat_num, val_rat_den, val_inf_num, val_inf_den);
                env->CallVoidMethod(c_obj, mthd_id, i_name, i_it->second);
            }
        }
        else if (const var_item *ei = dynamic_cast<const var_item *>(&itm))
        { // the expression represents an enum type..
            const auto vals = cr.get_ov_theory().value(ei->ev);
            jobjectArray vals_array = env->NewObjectArray(static_cast<jsize>(vals.size()), item_cls, NULL);
            size_t i = 0;
            for (const auto &v : vals)
                env->SetObjectArrayElement(vals_array, static_cast<jsize>(i++), all_items.at(static_cast<const item *>(v)));

            if (i_it == all_items.cend())
            { // we create a new enum..
                jobject c_type = all_types.at(&itm.get_type());
                jstring lit_s = env->NewStringUTF(("e" + std::to_string(ei->ev)).c_str());
                new_obj = env->NewGlobalRef(env->NewObject(enum_item_cls, enum_item_ctr_id, slv_obj, c_type, lit_s, vals_array));
                env->CallVoidMethod(c_obj, mthd_id, i_name, new_obj);
                all_items.emplace(ei, new_obj);
                env->DeleteLocalRef(lit_s);
            }
            else
            { // we update the value..
                env->CallVoidMethod(i_it->second, enum_item_set_mthd_id, vals_array);
                env->CallVoidMethod(c_obj, mthd_id, i_name, i_it->second);
            }

            env->DeleteLocalRef(vals_array);
        }
        else if (const string_item *si = dynamic_cast<const string_item *>(&itm))
        { // the expression represents a primitive string type..
            jstring c_val = env->NewStringUTF(si->get_value().c_str());
            if (i_it == all_items.cend())
            {
                new_obj = env->NewGlobalRef(env->NewObject(string_item_cls, string_item_ctr_id, slv_obj, c_val));
                env->CallVoidMethod(c_obj, mthd_id, i_name, new_obj);
                all_items.emplace(si, new_obj);
            }
            else
            { // we update the value..
                env->CallVoidMethod(i_it->second, string_item_set_mthd_id, c_val);
                env->CallVoidMethod(c_obj, mthd_id, i_name, i_it->second);
            }
            env->DeleteLocalRef(c_val);
        }
        else // the expression represents an item..
            env->CallVoidMethod(c_obj, mthd_id, i_name, i_it->second);
        env->DeleteLocalRef(i_name);
        return new_obj;
    }
} // namespace ratio
