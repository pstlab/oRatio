#include "lexer.h"
#include "parser.h"
#include "compilation_unit.h"
#include <sstream>
#include <cassert>

using namespace ratio;

void test_lexer()
{
    std::stringstream ss("real a = 5 + 2;\nfalse;");
    lexer l(ss);
    token *t0 = l.next();
    assert(t0->sym == REAL_ID);
    token *t1 = l.next();
    assert(t1->sym == ID_ID);
    token *t2 = l.next();
    assert(t2->sym == EQ_ID);
    token *t3 = l.next();
    assert(t3->sym == IntLiteral_ID);
    token *t4 = l.next();
    assert(t4->sym == PLUS_ID);
    token *t5 = l.next();
    assert(t5->sym == IntLiteral_ID);
    token *t6 = l.next();
    assert(t6->sym == SEMICOLON_ID);
    token *t7 = l.next();
    assert(t7->sym == FALSE_ID);
    token *t8 = l.next();
    assert(t8->sym == SEMICOLON_ID);
    token *t9 = l.next();
    assert(t9->sym == EOF_ID);
}

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

int main(int argc, char *argv[])
{
    test_lexer();
}