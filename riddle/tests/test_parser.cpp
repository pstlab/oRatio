#include "riddle_parser.h"
#include <sstream>
#include <cassert>

using namespace riddle;

void test_parser_0()
{
    std::stringstream ss("real a;\n1 <= a;");
    parser prs(ss);
    ast::compilation_unit *cu = prs.parse();
}

void test_parser_1()
{
    std::stringstream ss("real a = 5 +2;\nfalse;");
    parser prs(ss);
    ast::compilation_unit *cu = prs.parse();
}

void test_parser_2()
{
    std::stringstream ss("goal g0 = new At(l:5+3);");
    parser prs(ss);
    ast::compilation_unit *cu = prs.parse();
}

void test_parser_3()
{
    std::stringstream ss("enum Speed {\"High\", \"Medium\", \"Low\"}; Speed x3;");
    parser prs(ss);
    ast::compilation_unit *cu = prs.parse();
}

int main(int, char **)
{
    test_parser_0();
    test_parser_1();
    test_parser_2();
    test_parser_3();
}