#include "node.h"

namespace kb
{
    predicate_node::predicate_node(const std::string &id, const predicate &pred) : id(id), pred(pred) {}
    predicate_node::~predicate_node() {}
} // namespace kb
