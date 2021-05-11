#include "java_solver_listener.h"

using namespace ratio;

namespace ratio
{

    java_solver_listener::java_solver_listener(solver &s, JNIEnv *env, jobject obj) : scoped_env(env), solver_listener(s), slv_obj(env->NewGlobalRef(obj)), solver_cls(reinterpret_cast<jclass>(env->NewGlobalRef(env->GetObjectClass(obj)))),
                                                                                      flaw_created_mthd_id(env->GetMethodID(solver_cls, "fireFlawCreated", "(J[JLjava/lang/String;BII)V")),
                                                                                      flaw_state_changed_mthd_id(env->GetMethodID(solver_cls, "fireFlawStateChanged", "(JB)V")),
                                                                                      flaw_cost_changed_mthd_id(env->GetMethodID(solver_cls, "fireFlawCostChanged", "(JJJ)V")),
                                                                                      flaw_position_changed_mthd_id(env->GetMethodID(solver_cls, "fireFlawPositionChanged", "(JII)V")),
                                                                                      current_flaw_mthd_id(env->GetMethodID(solver_cls, "fireCurrentFlaw", "(J)V")),
                                                                                      resolver_created_mthd_id(env->GetMethodID(solver_cls, "fireResolverCreated", "(JJLjava/lang/String;JJB)V")),
                                                                                      resolver_state_changed_mthd_id(env->GetMethodID(solver_cls, "fireResolverStateChanged", "(JB)V")),
                                                                                      current_resolver_mthd_id(env->GetMethodID(solver_cls, "fireCurrentResolver", "(J)V")),
                                                                                      causal_link_added_mthd_id(env->GetMethodID(solver_cls, "fireCausalLinkAdded", "(JJ)V")) {}
    java_solver_listener::~java_solver_listener()
    {
        const auto &env = get_env();
        env->DeleteGlobalRef(slv_obj);
        env->DeleteGlobalRef(solver_cls);
    }

    void java_solver_listener::flaw_created(const flaw &f)
    {
        const auto &env = get_env();
        // the flaw's id..
        jlong id = reinterpret_cast<jlong>(&f);

        // the flaw's cause..
        std::vector<jlong> c_causes;
        c_causes.reserve(f.get_causes().size());
        for (const auto &c : f.get_causes())
            c_causes.push_back(reinterpret_cast<jlong>(c));
        jlongArray causes = env->NewLongArray(static_cast<jsize>(c_causes.size()));
        env->SetLongArrayRegion(causes, 0, static_cast<jsize>(c_causes.size()), c_causes.data());

        // the flaw's label..
        jstring label = env->NewStringUTF(f.get_label().c_str());

        // the flaw's state..
        jbyte state = static_cast<jbyte>(slv.get_sat_core().value(f.get_phi()));

        // the flaw's position..
        const auto [lb, ub] = slv.get_idl_theory().bounds(f.get_position());

        env->CallVoidMethod(slv_obj, flaw_created_mthd_id, id, causes, label, state, lb, ub);
        env->DeleteLocalRef(label);
        env->DeleteLocalRef(causes);
    }
    void java_solver_listener::flaw_state_changed(const flaw &f)
    {
        // the flaw's id..
        jlong id = reinterpret_cast<jlong>(&f);

        // the flaw's state..
        jbyte state = static_cast<jbyte>(slv.get_sat_core().value(f.get_phi()));

        get_env()->CallVoidMethod(slv_obj, flaw_state_changed_mthd_id, id, state);
    }
    void java_solver_listener::flaw_cost_changed(const flaw &f)
    {
        // the flaw's id..
        jlong id = reinterpret_cast<jlong>(&f);

        // the flaw's current estimated cost..
        const auto est_cost = f.get_estimated_cost();
        jlong cost_num = est_cost.numerator(), cost_den = est_cost.denominator();

        get_env()->CallVoidMethod(slv_obj, flaw_cost_changed_mthd_id, id, cost_num, cost_den);
    }
    void java_solver_listener::flaw_position_changed(const flaw &f)
    {
        // the flaw's id..
        jlong id = reinterpret_cast<jlong>(&f);

        // the flaw's position..
        const auto [lb, ub] = slv.get_idl_theory().bounds(f.get_position());
        jint jlb = lb, jub = ub;

        get_env()->CallVoidMethod(slv_obj, flaw_position_changed_mthd_id, id, jlb, jub);
    }
    void java_solver_listener::current_flaw(const flaw &f)
    {
        // the flaw's id..
        jlong id = reinterpret_cast<jlong>(&f);

        get_env()->CallVoidMethod(slv_obj, current_flaw_mthd_id, id);
    }

    void java_solver_listener::resolver_created(const resolver &r)
    {
        const auto &env = get_env();
        // the resolver's id..
        jlong id = reinterpret_cast<jlong>(&r);

        // the resolver's effect..
        jlong effect = reinterpret_cast<jlong>(&r.get_effect());

        // the resolver's label..
        jstring label = env->NewStringUTF(r.get_label().c_str());

        // the resolver's intrinsic cost..
        const auto est_cost = r.get_intrinsic_cost();
        jlong cost_num = est_cost.numerator(), cost_den = est_cost.denominator();

        // the resolver's state..
        jbyte state = static_cast<jbyte>(slv.get_sat_core().value(r.get_rho()));

        env->CallVoidMethod(slv_obj, resolver_created_mthd_id, id, effect, label, cost_num, cost_den, state);
        env->DeleteLocalRef(label);
    }
    void java_solver_listener::resolver_state_changed(const resolver &r)
    {
        // the resolver's id..
        jlong id = reinterpret_cast<jlong>(&r);

        // the resolver's state..
        jbyte state = static_cast<jbyte>(slv.get_sat_core().value(r.get_rho()));

        get_env()->CallVoidMethod(slv_obj, resolver_state_changed_mthd_id, id, state);
    }
    void java_solver_listener::current_resolver(const resolver &r)
    {
        // the resolver's id..
        jlong id = reinterpret_cast<jlong>(&r);

        get_env()->CallVoidMethod(slv_obj, current_resolver_mthd_id, id);
    }

    void java_solver_listener::causal_link_added(const flaw &f, const resolver &r)
    {
        // the flaw's id..
        jlong flaw_id = reinterpret_cast<jlong>(&f);

        // the resolver's id..
        jlong resolver_id = reinterpret_cast<jlong>(&r);

        get_env()->CallVoidMethod(slv_obj, causal_link_added_mthd_id, flaw_id, resolver_id);
    }
} // namespace ratio
