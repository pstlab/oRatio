#include "variable_declaration.h"
#include "expression.h"

namespace ratio
{

namespace ast
{
variable_declaration::variable_declaration(const std::string &n, const expression *const e) : name(n), xpr(e) {}
variable_declaration::~variable_declaration() { delete xpr; }
}
}