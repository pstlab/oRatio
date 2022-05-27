#include "rule_parser.h"
#include <sstream>
#include <cassert>

using namespace rbs;

void test_parser_0()
{
    std::stringstream ss("P0(p0:1, p1:1.5, p2:\"test\", p3: true); P1(p0:1, p1:1.5, p2:\"test\", p3: true);");
    parser p(ss);
    ast::compilation_unit *cu = p.parse();
}

void test_parser_1()
{
    std::stringstream ss("(r0 P0(p0:1, p1:1.5, p2:\"test\", p3: true) & P1(p0:1, p1:1.5, p2:\"test\", p3: true) => assert P1(p0:2, p1:2.5, p2:\"test\", p3: true))");
    parser p(ss);
    ast::compilation_unit *cu = p.parse();
}

int main(int, char **)
{
    test_parser_0();
    test_parser_1();
}