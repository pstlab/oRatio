#pragma once

namespace ratio
{

class scope
{
public:
  scope();
  scope(const scope &orig) = delete;
  ~scope();

private:
};
} // namespace ratio
