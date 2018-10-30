#include "type.h"
#include "item.h"
#include "core.h"

namespace ratio
{

type::type(core &cr, scope &scp, const std::string &name, bool primitive) : scope(cr, scp), name(name), primitive(primitive) {}

type::~type() {}

bool type::is_assignable_from(const type &t) const noexcept
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

expr type::new_instance(context &ctx)
{
    expr i = new item(cr, ctx, *this);
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

expr type::new_existential()
{
    if (instances.size() == 1)
        return *instances.begin();
    else
    {
        std::unordered_set<item *> c_items;
        for (const auto &i : instances)
            c_items.insert(&*i);
        return cr.new_enum(*this, c_items);
    }
}

field &type::get_field(const std::string &f_name) const
{
    const auto at_f = fields.find(f_name);
    if (at_f != fields.end())
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

bool_type::bool_type(core &cr) : type(cr, cr, BOOL_KEYWORD, true) {}
bool_type::~bool_type() {}

int_type::int_type(core &cr) : type(cr, cr, INT_KEYWORD, true) {}
int_type::~int_type() {}

real_type::real_type(core &cr) : type(cr, cr, REAL_KEYWORD, true) {}
real_type::~real_type() {}

string_type::string_type(core &cr) : type(cr, cr, STRING_KEYWORD, true) {}
string_type::~string_type() {}
} // namespace ratio
