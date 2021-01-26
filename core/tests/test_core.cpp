#include "core.h"
#include "item.h"
#include "combinations.h"
#include "cartesian_product.h"
#include <cassert>

using namespace ratio;

class test_solver : public core
{
    virtual void solve() override {}
    virtual void new_atom(atom &atm, const bool &is_fact) override {}
    virtual void new_disjunction(context &ctx, const std::vector<const conjunction *> &conjs) override {}
};

void test_core_0()
{
    test_solver s;
    bool_expr b0 = s.new_bool();
    arith_expr i0 = s.new_int();
    arith_expr r0 = s.new_real();

    smt::lbool b_val = s.bool_value(b0);
    assert(b_val == smt::Undefined);

    bool nc = s.get_sat_core().new_clause({b0->l});
    assert(nc);
    bool prop = s.get_sat_core().propagate();
    assert(prop);
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

    bool nc = s.get_sat_core().new_clause({b0->l});
    assert(nc);
    bool prop = s.get_sat_core().propagate();
    assert(prop);
    b_val = s.bool_value(b0);
    assert(b_val == smt::True);
}

void test_combinations()
{
    auto combs = combinations(std::vector<char>({'a', 'b', 'c', 'd'}), 3);
    assert(combs.size() == 4);
    assert(combs.at(0) == std::vector<char>({'a', 'b', 'c'}));
    assert(combs.at(1) == std::vector<char>({'a', 'b', 'd'}));
    assert(combs.at(2) == std::vector<char>({'a', 'c', 'd'}));
    assert(combs.at(3) == std::vector<char>({'b', 'c', 'd'}));
}

void test_cartesian_product()
{
    auto prod = cartesian_product(std::vector<std::vector<char>>({{'a', 'b'}, {'c', 'd'}}));
    assert(prod.size() == 4);
    assert(prod.at(0) == std::vector<char>({'a', 'c'}));
    assert(prod.at(1) == std::vector<char>({'a', 'd'}));
    assert(prod.at(2) == std::vector<char>({'b', 'c'}));
    assert(prod.at(3) == std::vector<char>({'b', 'd'}));
}

int main(int, char **)
{
    test_core_0();
    test_core_1();

    test_combinations();
    test_cartesian_product();
}