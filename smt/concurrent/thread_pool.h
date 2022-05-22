#pragma once

#include "concurrent_export.h"
#include <functional>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace smt
{
  class thread_pool
  {
  public:
    CONCURRENT_EXPORT thread_pool(const unsigned &c_size = std::thread::hardware_concurrency());
    thread_pool(const thread_pool &orig) = delete;
    CONCURRENT_EXPORT virtual ~thread_pool();

    inline size_t size() const { return workers.size(); }
    CONCURRENT_EXPORT void enqueue(std::function<void()> f);
    CONCURRENT_EXPORT void join();

  private:
    std::vector<std::thread> workers;
    size_t active = 0;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop = false;
  };
} // namespace smt