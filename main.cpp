#include <iostream>

int main(int argc, char *argv[])
{
    std::cout << "starting ratio";
#ifndef NDEBUG
    std::cout << " in debug mode";
#endif
    std::cout << ".." << std::endl;
}