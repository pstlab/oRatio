#include "typedef_type.h"
#include "core_parser.h"
#include "context.h"

namespace ratio
{
    typedef_type::typedef_type(core &cr, scope &scp, const std::string &name, const type &base_type, const riddle::ast::expression *const e) : type(cr, scp, name), base_type(base_type), xpr(e) {}
    typedef_type::~typedef_type() {}

    expr typedef_type::new_instance(context &ctx) { return dynamic_cast<const ast::expression *const>(xpr)->evaluate(*this, ctx); }
} // namespace ratio