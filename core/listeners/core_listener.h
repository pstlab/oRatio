#pragma once

namespace ratio
{

class core;
class method;
class type;
class predicate;
class constructor;

class core_listener
{
  friend class core;

public:
  core_listener(core &cr);
  core_listener(const core_listener &orig) = delete;
  virtual ~core_listener();

private:
  virtual void method_created(const method &m);
  virtual void method_created(const type &t, const method &m);

  virtual void type_created(const type &t);
  virtual void type_created(const type &et, const type &t);
  virtual void type_inherited(const type &st, const type &t);

  virtual void predicate_created(const predicate &p);
  virtual void predicate_created(const type &t, const predicate &p);

  virtual void constructor_created(const type &et, const constructor &c);

protected:
  core &cr;
};

} // namespace ratio
