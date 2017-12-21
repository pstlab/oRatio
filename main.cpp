#include <iostream>

int main(int argc, char *argv[])
{
    std::cout << "starting lucy";
#ifndef NDEBUG
    std::cout << " in debug mode";
#endif
    std::cout << ".." << std::endl;
}