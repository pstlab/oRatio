#include "type_declaration.h"

namespace ratio
{

namespace ast
{
type_declaration::type_declaration(const std::string &n) : name(n) {}
type_declaration::~type_declaration() {}
}
}