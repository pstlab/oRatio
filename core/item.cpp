#include "item.h"

namespace ratio
{

item::item(core &cr, const context ctx, const type &tp) : env(cr, ctx), tp(tp) {}

item::~item() {}
} // namespace ratio
