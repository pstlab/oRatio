#pragma once

namespace ratio
{

class core;
class type;

class core_listener
{
  friend void fire_new_type(const core &c, const type &t);

public:
  core_listener(core &cr);
  core_listener(const core_listener &orig) = delete;
  virtual ~core_listener();

private:
  virtual void type_created(const type &t);
  virtual void type_created(const type &et, const type &t);
  virtual void type_inherited(const type &st, const type &t);

protected:
  core &cr;
};

} // namespace ratio
