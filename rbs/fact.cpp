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
} // namespace kb
