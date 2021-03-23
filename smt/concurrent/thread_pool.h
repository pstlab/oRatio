#pragma once

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
    thread_pool(const unsigned &c_size = std::thread::hardware_concurrency());
    thread_pool(const thread_pool &orig) = delete;
    virtual ~thread_pool();

    inline size_t size() const { return workers.size(); }
    void enqueue(std::function<void()> f);
    void join();

  private:
    std::vector<std::thread> workers;
    size_t active = 0;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop = false;
  };
} // namespace smt