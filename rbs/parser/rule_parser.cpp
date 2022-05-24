#include "rule_parser.h"
#include <cassert>

namespace rbs
{
    using namespace ast;

    RBS_EXPORT parser::parser(std::istream &is) : lex(is) {}
    RBS_EXPORT parser::~parser() {}
} // namespace rbs