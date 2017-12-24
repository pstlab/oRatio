#include "typedef_type.h"
#include "context.h"
#include "expression.h"

namespace ratio
{

typedef_type::typedef_type(core &cr, scope &scp, const std::string &name, const type &base_type, const ast::expression *const e) : type(cr, scp, name), base_type(base_type), xpr(e) {}
typedef_type::~typedef_type() {}

expr typedef_type::new_instance(context &ctx) { return xpr->evaluate(*this, ctx); }
}