#include "solver.h"
#include "atom.h"
#include <cassert>

using namespace ratio;

void test_solver_0()
{
    solver s;
    s.init();
    s.read("bool b0; int i0; real r0;");

    bool_expr b0 = s.get("b0");

    smt::lbool b_val = s.bool_value(b0);
    assert(b_val == smt::Undefined);

    s.assert_facts({b0->l});
    b_val = s.bool_value(b0);
    assert(b_val == smt::True);
}

int main(int, char **)
{
    test_solver_0();
}