#include "lexer.h"
#include "parser.h"
#include <sstream>
#include <cassert>

using namespace ratio;

void test_lexer()
{
    lexer l(std::stringstream("real a = 5 + 2;\nfalse;"));
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

int main(int argc, char *argv[])
{
    test_lexer();
}