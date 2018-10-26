#include "core.h"
#include <cassert>

using namespace ratio;

void test_core_0()
{
    core c;
    bool_expr b0 = c.new_bool();
}

int main(int, char **)
{
    test_core_0();
}