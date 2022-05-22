#include "conjunction.h"
#include "env.h"
#include "core_parser.h"

namespace ratio
{
    conjunction::conjunction(scope &scp, const smt::rational &cst, std::vector<const riddle::ast::statement *> stmnts) : scope(scp), cost(cst), statements(std::move(stmnts)) {}

    CORE_EXPORT void conjunction::apply(context &ctx)
    {
        context c_ctx(new env(get_core(), context(ctx)));
        for (const auto &s : statements)
            dynamic_cast<const ast::statement *>(s)->execute(*this, c_ctx);
    }
} // namespace ratio