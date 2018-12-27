#pragma once

namespace ratio
{

class core;
class type;
class constructor;
class method;
class predicate;
class scope;
class field;
class item;

class core_listener
{
  friend class core;

public:
  core_listener(core &cr);
  core_listener(const core_listener &orig) = delete;
  virtual ~core_listener();

private:
  void new_type(const type &tp);
  void new_type(const type &tp_outer, const type &tp_inner);

  void new_constructor(const type &tp, const constructor &ctr);

  void new_method(const method &m);
  void new_method(const type &tp, const method &m);

  void new_predicate(const predicate &p);
  void new_predicate(const type &tp, const predicate &p);

  void new_field(const field &f);
  void new_field(const scope &scp, const field &f);

  void new_instance(const type &tp, const item &i);

protected:
  core &cr;
};
} // namespace ratio
