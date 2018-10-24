#pragma once

namespace ratio
{

class env;

class context
{
public:
  context(env *const ptr);
  context(const context &orig);
  virtual ~context();

  env &operator*() const { return *ptr; }
  env *operator->() const { return ptr; }

  bool operator==(const context &right) const { return ptr == right.ptr; }
  bool operator!=(const context &right) const { return !(*this == right); }

protected:
  env *const ptr;
};
} // namespace ratio
