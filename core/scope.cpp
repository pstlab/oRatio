#include "scope.h"

namespace ratio
{

scope::scope(core &cr, scope &scp) : cr(cr), scp(scp) {}

scope::~scope() {}
} // namespace ratio
