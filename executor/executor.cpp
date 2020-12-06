#include "executor.h"
#include <thread>
#include <chrono>

namespace ratio
{

    executor::executor(core &cr) : core_listener(cr) {}
    executor::~executor() {}

    void executor::set_interval(size_t interval)
    {
        executing = true;
        std::chrono::high_resolution_clock::time_point start;
        std::chrono::high_resolution_clock::time_point end;
        std::thread t([this, &start, &end, &interval]() {
            while (true)
            {
                if (!executing)
                    return;
                std::this_thread::sleep_for(std::chrono::milliseconds(interval) - (end - start));
                if (!executing)
                    return;
                start = std::chrono::high_resolution_clock::now();

                starting_atoms.clear();
                ending_atoms.clear();
                pulses.clear();

                // todo: do something..
                end = std::chrono::high_resolution_clock::now();
            }
        });
        t.detach();
    }

    void executor::set_timeout(size_t delay)
    {
        executing = true;
        std::thread t([this, delay]() {
            if (!executing)
                return;
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            if (!executing)
                return;
            // todo: do something..
        });
        t.detach();
    }

    void executor::stop() { executing = false; }

    void executor::state_changed()
    {
        cr.get_types();
    }
} // namespace ratio
