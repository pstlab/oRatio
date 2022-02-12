#pragma once

#include "core_listener.h"

namespace ratio
{
  class type;
  class predicate;

  class gui_core_listener : public core_listener
  {
  public:
    gui_core_listener(core &cr);

  private:
    void log(const std::string &msg) override;
    void read(const std::string &script) override;
    void read(const std::vector<std::string> &files) override;

    void state_changed() override;

    void started_solving() override;
    void solution_found() override;
    void inconsistent_problem() override;
  };
} // namespace ratio