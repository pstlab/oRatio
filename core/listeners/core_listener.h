#pragma once

namespace ratio
{

class core;
class type;

class core_listener
{
  friend class core;

public:
  core_listener(core &cr);
  core_listener(const core_listener &orig) = delete;
  virtual ~core_listener();

private:
  virtual void type_created(const type &t);

protected:
  core &cr;
};

} // namespace ratio
