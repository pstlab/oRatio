#include "riddle_lexer.h"
#include <sstream>
#include <cassert>

using namespace riddle;

void test_lexer_0()
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
    assert(t7->sym == BoolLiteral_ID);
    token *t8 = l.next();
    assert(t8->sym == SEMICOLON_ID);
    token *t9 = l.next();
    assert(t9->sym == EOF_ID);
}

int main(int, char **)
{
    test_lexer_0();
}