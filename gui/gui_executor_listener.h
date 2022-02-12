#pragma once

#include "executor_listener.h"

namespace ratio
{

  class gui_executor_listener : public executor_listener
  {
  public:
    gui_executor_listener(executor &e);
    ~gui_executor_listener();

  private:
    void tick(const smt::rational &time) override;
    void starting(const std::unordered_set<atom *> &atoms) override;
    void start(const std::unordered_set<atom *> &atoms) override;
    void ending(const std::unordered_set<atom *> &atoms) override;
    void end(const std::unordered_set<atom *> &atoms) override;
  };
} // namespace ratio