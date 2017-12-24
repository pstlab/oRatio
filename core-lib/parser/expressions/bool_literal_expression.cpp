#include "bool_literal_expression.h"
#include "context.h"

namespace ratio
{

namespace ast
{

bool_literal_expression::bool_literal_expression(const bool &l) : literal(l) {}
bool_literal_expression::~bool_literal_expression() {}

expr bool_literal_expression::evaluate(const scope &scp, context &) const { return scp.get_core().new_bool(literal); }
}
}