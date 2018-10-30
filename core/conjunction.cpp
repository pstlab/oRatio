#include "conjunction.h"
#include "env.h"

namespace ratio
{

conjunction::conjunction(core &cr, scope &scp, const smt::rational &cst) : scope(cr, scp), cost(cst) {}
conjunction::~conjunction() {}

void conjunction::apply(context &ctx) const
{
    // TODO: execute the statements..
}
}