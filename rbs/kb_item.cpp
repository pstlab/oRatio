#include "kb_item.h"
#include <algorithm>

namespace kb
{
    bool_item::bool_item(bool val) : val(val) {}
    bool bool_item::eq(const item &it) const noexcept
    {
        if (const bool_item *bi = dynamic_cast<const bool_item *>(&it))
            return val == bi->val;
        else
            return false;
    }

    arith_item::arith_item(smt::rational val) : val(val) {}
    bool arith_item::eq(const item &it) const noexcept
    {
        if (const arith_item *ai = dynamic_cast<const arith_item *>(&it))
            return val == ai->val;
        else
            return false;
    }

    string_item::string_item(const std::string &val) : val(val) {}
    bool string_item::eq(const item &it) const noexcept
    {
        if (const string_item *si = dynamic_cast<const string_item *>(&it))
            return val == si->val;
        else
            return false;
    }
} // namespace kb
