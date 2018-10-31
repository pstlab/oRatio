#include "core.h"
#include "item.h"
#include <cassert>

using namespace ratio;

void test_core_0()
{
    core c;
    bool_expr b0 = c.new_bool();
    arith_expr i0 = c.new_int();
    arith_expr r0 = c.new_real();

    smt::lbool b_val = c.bool_value(b0);
    assert(b_val == smt::Undefined);

    c.assert_facts({b0->l});
    b_val = c.bool_value(b0);
    assert(b_val == smt::True);
}

void test_core_1()
{
    core c;
    c.read("bool b0; int i0; real r0;");
}

int main(int, char **)
{
    test_core_0();
    test_core_1();
}