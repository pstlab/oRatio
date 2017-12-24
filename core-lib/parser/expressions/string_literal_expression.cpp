#include "string_literal_expression.h"
#include "core.h"

namespace ratio
{

namespace ast
{

string_literal_expression::string_literal_expression(const std::string &l) : literal(l) {}
string_literal_expression::~string_literal_expression() {}

expr string_literal_expression::evaluate(const scope &scp, context &) const { return scp.get_core().new_string(literal); }
}
}