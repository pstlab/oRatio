#include "rule_parser.h"
#include <sstream>
#include <cassert>

using namespace rbs;

void test_parser_0()
{
    std::stringstream ss("P0(p0:1, p1:1.5, p2:\"test\", p3: true);");
    parser p(ss);
    ast::compilation_unit *cu = p.parse();
}

int main(int, char **)
{
    test_parser_0();
}