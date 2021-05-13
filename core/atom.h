#pragma once

#include "item.h"

namespace ratio
{
  class predicate;

  class atom : public item
  {
  public:
    atom(core &cr, const context ctx, const predicate &pred);
    atom(const atom &orig) = delete;
    virtual ~atom();

    inline smt::var get_sigma() const noexcept { return sigma; } // returns the variable that represents the state of the atom: if the variable is true, the atom is active; if the variable is false, the atom is unified; if the variable is undefined, the atom is not justified..

    smt::lit new_eq(item &i) noexcept override;
    bool equates(const item &i) const noexcept override;

    smt::json to_json() const noexcept override;

  private:
    const smt::var sigma; // this variable represents the state of the atom: if the variable is true, the atom is active; if the variable is false, the atom is unified; if the variable is undefined, the atom is not justified..
  };
} // namespace ratio