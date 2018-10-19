#include "sat_core.h"
#include <iostream>

int main(int, char **)
{
    std::cout << "Hello, world!\n";
    int i = 10;
    std::cout << i << std::endl;

    smt::lit l0(1);
    smt::lit l1 = !l0;

    const smt::var &v = l0.get_var();

    std::vector<smt::lit> lits;
}
