#include "sat_core.h"
#include <cassert>

using namespace smt;

void test_basic_core()
{
    sat_core core;
    var b0 = core.new_var();
    var b1 = core.new_var();
    var b2 = core.new_var();

    bool nc = core.new_clause({lit(b0, false), !lit(b1), b2});
    assert(nc);
    bool ch = core.check();
    assert(ch);
    assert(core.value(b0) == Undefined);
    assert(core.value(b1) == Undefined);
    assert(core.value(b2) == Undefined);

    bool assm = core.assume(b0) && core.check();
    assert(assm);
    assert(core.value(b0) == True);
    assert(core.value(b1) == Undefined);
    assert(core.value(b2) == Undefined);

    assm = core.assume(b1) && core.check();
    assert(assm);
    assert(core.value(b0) == True);
    assert(core.value(b1) == True);
    assert(core.value(b2) == True);
}

void test_no_good()
{
    sat_core core;

    var b0 = core.new_var();
    var b1 = core.new_var();
    var b2 = core.new_var();
    var b3 = core.new_var();
    var b4 = core.new_var();
    var b5 = core.new_var();
    var b6 = core.new_var();
    var b7 = core.new_var();
    var b8 = core.new_var();

    bool nc = core.new_clause({b0, b1});
    assert(nc);
    nc = core.new_clause({b0, b1, b6});
    assert(nc);
    nc = core.new_clause({lit(b1, false), lit(b2, false), b3});
    assert(nc);
    nc = core.new_clause({lit(b3, false), b4, b7});
    assert(nc);
    nc = core.new_clause({lit(b3, false), b5, b8});
    assert(nc);
    nc = core.new_clause({lit(b4, false), lit(b5, false)});
    assert(nc);

    bool assm = core.assume(lit(b6, false)) && core.check();
    assert(assm);
    assm = core.assume(lit(b7, false)) && core.check();
    assert(assm);
    assm = core.assume(lit(b8, false)) && core.check();
    assert(assm);
    assm = core.assume(lit(b0, false)) && core.check();
    assert(assm);
}

int main(int argc, char *argv[])
{
    test_basic_core();
    test_no_good();
}