#include "node.h"

namespace kb
{
    predicate_node::predicate_node(const predicate &pred) : pred(pred) {}
    predicate_node::~predicate_node() {}
} // namespace kb
