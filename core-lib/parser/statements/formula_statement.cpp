#include "formula_statement.h"

namespace ratio
{

namespace ast
{

formula_statement::formula_statement(const bool &isf, const std::string &fn, const std::vector<std::string> &scp, const std::string &pn, const std::vector<std::pair<std::string, const expression *>> &assns) : is_fact(isf), formula_name(fn), formula_scope(scp), predicate_name(pn), assignments(assns) {}
formula_statement::~formula_statement()
{
    for (const auto &asgnmnt : assignments)
        delete asgnmnt.second;
}

void formula_statement::execute(const scope &scp, context &ctx) const
{
    predicate *p = nullptr;
    std::unordered_map<std::string, expr> assgnments;
    if (!formula_scope.empty()) // the scope is explicitely declared..
    {
        env *c_scope = &*ctx;
        for (const auto &s : formula_scope)
            c_scope = &*c_scope->get(s);
        p = &static_cast<item *>(c_scope)->tp.get_predicate(predicate_name);

        if (var_item *ee = dynamic_cast<var_item *>(c_scope)) // the scope is an enumerative expression..
            assgnments.insert({TAU, ee});
        else // the scope is a single item..
            assgnments.insert({TAU, context(c_scope)});
    }
    else
    {
        p = &scp.get_predicate(predicate_name);
        if (&p->get_scope() != &scp.get_core()) // we inherit the scope..
            assgnments.insert({TAU, ctx->get(TAU)});
    }

    for (const auto &a : assignments)
    {
        expr e = a.second->evaluate(scp, ctx);
        const type &tt = p->get_field(a.first).tp; // the target type..
        if (tt.is_assignable_from(e->tp))          // the target type is a superclass of the assignment..
            assgnments.insert({a.first, e});
        else if (e->tp.is_assignable_from(tt))                // the target type is a subclass of the assignment..
            if (var_item *ae = dynamic_cast<var_item *>(&*e)) // some of the allowed values might be inhibited..
            {
                std::unordered_set<var_value *> alwd_vals = scp.get_core().ov_th.value(ae->ev); // the allowed values..
                std::vector<lit> not_alwd_vals;                                                 // the not allowed values..
                for (const auto &ev : alwd_vals)
                    if (!tt.is_assignable_from(static_cast<item *>(ev)->tp)) // the target type is not a superclass of the value..
                        not_alwd_vals.push_back(lit(scp.get_core().ov_th.allows(ae->ev, *ev), false));
                if (alwd_vals.size() == not_alwd_vals.size()) // none of the values is allowed..
                    throw inconsistency_exception();          // no need to go further..
                else
                    scp.get_core().assert_facts(not_alwd_vals); // we inhibit the not allowed values..
            }
            else
                throw inconsistency_exception();
        else
            throw inconsistency_exception();
    }

    atom *a;
    if (assgnments.find(TAU) == assgnments.end())
    {
        // the new atom's scope is the core..
        context c_scope = &scp.get_core();
        a = static_cast<atom *>(&*p->new_instance(c_scope));
    }
    else
    {
        // we have computed the new atom's scope above..
        context c_scope = assgnments.at(TAU);
        a = static_cast<atom *>(&*p->new_instance(c_scope));
    }

    // we assign fields..
    a->items.insert(assgnments.begin(), assgnments.end());

    // we initialize atom's fields..
    std::queue<predicate *> q;
    q.push(p);
    while (!q.empty())
    {
        for (const auto &arg : q.front()->get_args())
        {
            if (a->items.find(arg->name) == a->items.end())
            {
                // the field is uninstantiated..
                type &tp = const_cast<type &>(arg->tp);
                if (tp.primitive)
                    a->items.insert({arg->name, tp.new_instance(ctx)});
                else
                    a->items.insert({arg->name, tp.new_existential()});
            }
        }
        for (const auto &sp : q.front()->get_supertypes())
            q.push(static_cast<predicate *>(sp));
        q.pop();
    }

    if (is_fact)
        scp.get_core().new_fact(*a);
    else
        scp.get_core().new_goal(*a);

    ctx->items.insert({formula_name, expr(a)});
}
}
}