#include "item.h"

namespace ratio
{

item::item(core &cr, const type &tp) : env(cr), tp(tp) {}

item::~item() {}
} // namespace ratio
