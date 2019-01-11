#include "core_listener.h"
#include "core.h"

namespace ratio
{

core_listener::core_listener(core &cr) : cr(cr) { cr.listeners.push_back(this); }
core_listener::~core_listener() { cr.listeners.erase(std::find(cr.listeners.begin(), cr.listeners.end(), this)); }

void core_listener::method_created(const method &m) {}
void core_listener::method_created(const type &et, const method &m) {}

void core_listener::type_created(const type &t) {}
void core_listener::type_created(const type &et, const type &t) {}
void core_listener::type_inherited(const type &st, const type &t) {}

void core_listener::predicate_created(const predicate &p) {}
void core_listener::predicate_created(const type &t, const predicate &p) {}

void core_listener::constructor_created(const type &et, const constructor &c) {}
} // namespace ratio
