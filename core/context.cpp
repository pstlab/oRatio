#include "context.h"
#include "env.h"

namespace ratio
{

context::context(env *const ptr) : ptr(ptr) { ptr->ref_count++; }
context::context(const context &orig) : ptr(orig.ptr) { ptr->ref_count++; }
context::~context()
{
    ptr->ref_count--;
    if (ptr->ref_count == 0)
    {
        delete ptr;
    }
}
} // namespace ratio
