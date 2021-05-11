#include "type.h"
#include "item.h"
#include "core.h"
#include "field.h"
#include "constructor.h"
#include "method.h"
#include "predicate.h"
#include "riddle_lexer.h"
#include <algorithm>
#include <stdexcept>
#include <cassert>

namespace ratio
{
    CORE_EXPORT type::type(core &cr, scope &scp, const std::string &name, bool primitive) : scope(cr, scp), name(name), primitive(primitive) {}
    CORE_EXPORT type::~type()
    {
        // we delete the predicates..
        for (const auto &[pred_name, pred] : predicates)
            delete pred;

        // we delete the types..
        for (const auto &[tp_name, tp] : types)
            delete tp;

        // we delete the methods..
        for (const auto &[mthd_name, mthds] : methods)
            for (const auto &m : mthds)
                delete m;

        // we delete the constructors..
        for (const auto &c : constructors)
            delete c;
    }

    CORE_EXPORT bool type::is_assignable_from(const type &t) const noexcept
    {
        std::queue<const type *> q;
        q.push(&t);
        while (!q.empty())
        {
            if (q.front() == this)
                return true;
            else
            {
                for (const auto &st : q.front()->supertypes)
                    q.push(st);
                q.pop();
            }
        }
        return false;
    }

    CORE_EXPORT expr type::new_instance(context &ctx)
    {
        expr i = new item(cr, context(ctx), *this);
        std::queue<type *> q;
        q.push(this);
        while (!q.empty())
        {
            q.front()->instances.push_back(i);
            for (const auto &st : q.front()->supertypes)
                q.push(st);
            q.pop();
        }

        return i;
    }

    CORE_EXPORT expr type::new_existential()
    {
        assert(!instances.empty());
        if (instances.size() == 1)
            return *instances.cbegin();
        else
        {
            std::vector<item *> c_items;
            c_items.reserve(instances.size());
            for (const auto &i : instances)
                c_items.push_back(&*i);
            return cr.new_enum(*this, c_items);
        }
    }

    CORE_EXPORT void type::new_supertypes(type &t, const std::vector<type *> &sts) noexcept
    {
        for (const auto &st : sts)
            t.supertypes.push_back(st);
    }
    CORE_EXPORT void type::new_supertypes(const std::vector<type *> &sts) noexcept
    {
        for (const auto &st : sts)
            supertypes.push_back(st);
    }

    CORE_EXPORT void type::new_constructors(const std::vector<const constructor *> &cs) noexcept
    {
        for (const auto &c : cs)
            constructors.push_back(c);
    }

    void type::new_methods(const std::vector<const method *> &ms) noexcept
    {
        for (const auto &m : ms)
            methods[m->get_name()].push_back(m);
    }

    void type::new_types(const std::vector<type *> &ts) noexcept
    {
        for (const auto &t : ts)
            types.emplace(t->name, t);
    }

    CORE_EXPORT void type::new_predicates(const std::vector<predicate *> &ps, bool notify) noexcept
    {
        for (const auto &p : ps)
        {
            predicates.emplace(p->get_name(), p);

            if (notify)
            {
                // we notify all the supertypes that a new predicate has been created..
                std::queue<type *> q;
                q.push(this);
                while (!q.empty())
                {
                    q.front()->new_predicate(*p);
                    for (const auto &st : q.front()->supertypes)
                        q.push(st);
                    q.pop();
                }
            }
        }
    }

    const constructor &type::get_constructor(const std::vector<const type *> &ts) const
    {
        assert(std::none_of(ts.cbegin(), ts.cend(), [](const type *t) { return t == nullptr; }));
        bool found = false;
        for (const auto &cnstr : constructors)
            if (cnstr->args.size() == ts.size())
            {
                found = true;
                for (unsigned int i = 0; i < ts.size(); i++)
                    if (!cnstr->args[i]->get_type().is_assignable_from(*ts[i]))
                    {
                        found = false;
                        break;
                    }
                if (found)
                    return *cnstr;
            }

        throw std::out_of_range(name);
    }

    CORE_EXPORT const field &type::get_field(const std::string &f_name) const
    {
        if (const auto at_f = fields.find(f_name); at_f != fields.cend())
            return *at_f->second;

        // if not here, check any enclosing scope
        try
        {
            return scp.get_field(f_name);
        }
        catch (const std::out_of_range &)
        {
            // if not in any enclosing scope, check any superclass
            for (const auto &st : supertypes)
                try
                {
                    return st->get_field(f_name);
                }
                catch (const std::out_of_range &)
                {
                }
        }

        // not found
        throw std::out_of_range(f_name);
    }

    CORE_EXPORT const method &type::get_method(const std::string &m_name, const std::vector<const type *> &ts) const
    {
        const auto at_m = methods.find(m_name);
        if (at_m != methods.cend())
        {
            bool found = false;
            for (const auto &mthd : at_m->second)
                if (mthd->args.size() == ts.size())
                {
                    found = true;
                    for (unsigned int i = 0; i < ts.size(); i++)
                        if (!mthd->args[i]->get_type().is_assignable_from(*ts[i]))
                        {
                            found = false;
                            break;
                        }
                    if (found)
                        return *mthd;
                }
        }

        // if not here, check any enclosing scope
        try
        {
            return scp.get_method(m_name, ts);
        }
        catch (const std::out_of_range &)
        {
            // if not in any enclosing scope, check any superclass
            for (const auto &st : supertypes)
            {
                try
                {
                    return st->get_method(m_name, ts);
                }
                catch (const std::out_of_range &)
                {
                }
            }
        }

        // not found
        throw std::out_of_range(m_name);
    }

    CORE_EXPORT type &type::get_type(const std::string &t_name) const
    {
        if (const auto at_tp = types.find(t_name); at_tp != types.cend())
            return *at_tp->second;

        // if not here, check any enclosing scope
        try
        {
            return scp.get_type(t_name);
        }
        catch (const std::out_of_range &)
        {
            // if not in any enclosing scope, check any superclass
            for (const auto &st : supertypes)
            {
                try
                {
                    return st->get_type(t_name);
                }
                catch (const std::out_of_range &)
                {
                }
            }
        }

        // not found
        throw std::out_of_range(t_name);
    }

    CORE_EXPORT predicate &type::get_predicate(const std::string &p_name) const
    {
        if (const auto at_p = predicates.find(p_name); at_p != predicates.cend())
            return *at_p->second;

        // if not here, check any enclosing scope
        try
        {
            return scp.get_predicate(p_name);
        }
        catch (const std::out_of_range &)
        {
            // if not in any enclosing scope, check any superclass
            for (const auto &st : supertypes)
            {
                try
                {
                    return st->get_predicate(p_name);
                }
                catch (const std::out_of_range &)
                {
                }
            }
        }

        // not found
        throw std::out_of_range(p_name);
    }

    bool_type::bool_type(core &cr) : type(cr, cr, BOOL_KEYWORD, true) {}
    bool_type::~bool_type() {}
    expr bool_type::new_instance(context &) noexcept { return get_core().new_bool(); }

    int_type::int_type(core &cr) : type(cr, cr, INT_KEYWORD, true) {}
    int_type::~int_type() {}
    bool int_type::is_assignable_from(const type &t) const noexcept { return &t == this || &t == &get_core().get_type(TP_KEYWORD); }
    expr int_type::new_instance(context &) noexcept { return get_core().new_int(); }

    real_type::real_type(core &cr) : type(cr, cr, REAL_KEYWORD, true) {}
    real_type::~real_type() {}
    bool real_type::is_assignable_from(const type &t) const noexcept { return &t == this || &t == &get_core().get_type(TP_KEYWORD); }
    expr real_type::new_instance(context &) noexcept { return get_core().new_real(); }

    tp_type::tp_type(core &cr) : type(cr, cr, TP_KEYWORD, true) {}
    tp_type::~tp_type() {}
    bool tp_type::is_assignable_from(const type &t) const noexcept { return &t == this || &t == &get_core().get_type(INT_KEYWORD) || &t == &get_core().get_type(REAL_KEYWORD); }
    expr tp_type::new_instance(context &) noexcept { return get_core().new_tp(); }

    string_type::string_type(core &cr) : type(cr, cr, STRING_KEYWORD, true) {}
    string_type::~string_type() {}
    expr string_type::new_instance(context &) noexcept { return get_core().new_string(); }
} // namespace ratio