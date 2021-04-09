#include "conjunction.h"
#include "env.h"
#include "core_parser.h"

namespace ratio
{
    conjunction::conjunction(core &cr, scope &scp, const smt::rational &cst, const std::vector<const riddle::ast::statement *> &stmnts) : scope(cr, scp), cost(cst), statements(stmnts) {}
    conjunction::~conjunction() {}

    CORE_EXPORT void conjunction::apply(context &ctx) const
    {
        context c_ctx(new env(get_core(), context(ctx)));
        for (const auto &s : statements)
            dynamic_cast<const ast::statement *>(s)->execute(*this, c_ctx);
    }
} // namespace ratio