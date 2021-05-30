#include "config.h"
#include "solver.h"
#include <cassert>

using namespace ratio;

void test_config()
{
    config cnf;
    solver slv;
    slv.read("predicate P0(){} predicate P1(){} class C { predicate P2(){}}");
    cnf.read(slv, "{\"notify-start\": [\"P0\", \"C.P2\"]}");
}

int main(int, char **)
{
    test_config();
}