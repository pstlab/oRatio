#pragma once

#include <thread>
#include <chrono>
#include <mutex>

namespace ratio
{
  template <typename function>
  class timer
  {
  public:
    timer(const size_t &tick_dur, function f) : tick_duration(tick_dur), fun(f) {}
    ~timer() {}

  public:
    void lock() { mtx.lock(); }
    void unlock() { mtx.unlock(); }

    std::thread start()
    {
      mtx.lock();
      executing = true;
      mtx.unlock();
      tick_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(tick_duration);
      std::thread t([this]() {
        while (true)
        {
          if (!executing)
            return;
          std::this_thread::sleep_until(tick_time);
          if (!executing)
            return;
          tick_time += std::chrono::milliseconds(tick_duration);
          mtx.lock();
          fun();
          mtx.unlock();
        }
      });
      return t;
    }

    void stop()
    {
      mtx.lock();
      executing = false;
      mtx.unlock();
    }

  private:
    const size_t tick_duration; // the duration of each tick in milliseconds..
    std::chrono::steady_clock::time_point tick_time;
    std::mutex mtx; // a mutex for the critical sections..
    bool executing = false;
    function fun;
  };
} // namespace ratio
