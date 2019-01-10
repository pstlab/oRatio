#include "core_listener.h"
#include "core.h"

namespace ratio
{

core_listener::core_listener(core &cr) : cr(cr) { cr.listeners.push_back(this); }
core_listener::~core_listener() { cr.listeners.erase(std::find(cr.listeners.begin(), cr.listeners.end(), this)); }

void core_listener::type_created(const type &t) {}
} // namespace ratio
