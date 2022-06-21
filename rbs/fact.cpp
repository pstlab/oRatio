#include "fact.h"

namespace kb
{
    predicate::predicate(const std::string &name) : name(name) {}
    predicate::~predicate() {}

    fact &predicate::new_instance() noexcept
    {
        fact *f = new fact(*this);
        instances.insert(f);
        return *f;
    }

    fact::fact(predicate &p) : pred(p) {}
    fact::~fact() { pred.instances.erase(this); }

    void fact::set(const std::string &fn, expr xpr)
    {
        xprs[fn] = xpr;
    }

    bool fact::eq(const item &it) const noexcept
    {
        if (const fact *fi = dynamic_cast<const fact *>(&it))
        {
            for (const auto &xpr : fi->xprs)
                if (const auto &xpr_it = xprs.find(xpr.first); xpr_it != xprs.end())
                    if (!xpr_it->second->eq(*xpr.second.get()))
                        return false;
            return true;
        }
        else
            return false;
    }
} // namespace kb
