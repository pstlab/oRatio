#include "atom.h"
#include "predicate.h"
#include "core.h"

namespace ratio
{

atom::atom(core &cr, const context ctx, const predicate &pred) : item(cr, ctx, pred), sigma(cr.sat_cr.new_var()) {}
atom::~atom() {}
}