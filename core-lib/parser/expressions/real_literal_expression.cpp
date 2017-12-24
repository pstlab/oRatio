#include "real_literal_expression.h"
#include "context.h"

namespace ratio
{

namespace ast
{

real_literal_expression::real_literal_expression(const smt::rational &l) : literal(l) {}
real_literal_expression::~real_literal_expression() {}

expr real_literal_expression::evaluate(const scope &scp, context &) const { return scp.get_core().new_real(literal); }
}
}