#include "int_literal_expression.h"
#include "context.h"

namespace ratio
{

namespace ast
{

int_literal_expression::int_literal_expression(const I &l) : literal(l) {}
int_literal_expression::~int_literal_expression() {}

expr int_literal_expression::evaluate(const scope &scp, context &) const { return scp.get_core().new_int(literal); }
}
}