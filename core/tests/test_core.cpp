#include "core.h"
#include "item.h"
#include <cassert>

using namespace ratio;

class test_solver : public core
{
    virtual void solve() override {}
    virtual void new_fact(atom &atm) override {}
    virtual void new_goal(atom &atm) override {}
    virtual void new_disjunction(context &ctx, const disjunction &disj) override {}
};

void test_core_0()
{
    test_solver s;
    bool_expr b0 = s.new_bool();
    arith_expr i0 = s.new_int();
    arith_expr r0 = s.new_real();

    smt::lbool b_val = s.bool_value(b0);
    assert(b_val == smt::Undefined);

    s.assert_facts({b0->l});
    b_val = s.bool_value(b0);
    assert(b_val == smt::True);
}

void test_core_1()
{
    test_solver s;
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
    test_core_0();
    test_core_1();
}