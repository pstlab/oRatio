#include "type.h"

namespace ratio
{

type::type(core &cr, scope &scp, const std::string &name, bool primitive) : scope(cr, scp), name(name), primitive(primitive) {}

type::~type() {}
} // namespace ratio
