#pragma once

#include "core.h"

namespace ratio
{

class core_listener
{
  friend class core;

public:
  core_listener(core &cr) : cr(cr) { cr.listeners.push_back(this); }
  core_listener(const core_listener &orig) = delete;
  virtual ~core_listener() { cr.listeners.erase(std::find(cr.listeners.begin(), cr.listeners.end(), this)); }

private:
  virtual void method_created(const method &m) {}
  virtual void method_created(const type &t, const method &m) {}

  virtual void type_created(const type &t) {}
  virtual void type_created(const type &et, const type &t) {}
  virtual void type_inherited(const type &st, const type &t) {}

  virtual void predicate_created(const predicate &p) {}
  virtual void predicate_created(const type &t, const predicate &p) {}

  virtual void constructor_created(const type &et, const constructor &c) {}

  virtual void field_created(const scope &sc, const field &f) {}

protected:
  core &cr;
};

} // namespace ratio
