#include "conjunction_statement.h"

namespace ratio
{

namespace ast
{

conjunction_statement::conjunction_statement(const std::vector<const statement *> &stmnts) : statements(stmnts) {}
conjunction_statement::~conjunction_statement()
{
    for (const auto &st : statements)
        delete st;
}

void conjunction_statement::execute(const scope &scp, context &ctx) const
{
    for (const auto &st : statements)
        st->execute(scp, ctx);
}
}
}