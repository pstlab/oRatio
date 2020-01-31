#include "thread_pool.h"
#include <cassert>

namespace smt
{

thread_pool::thread_pool()
{
    unsigned c_size = std::thread::hardware_concurrency();
    workers.reserve(c_size);
    for (unsigned i = 0; i < c_size; i++)
    {
        workers.emplace_back([this, i] {
            while (true)
            {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(this->queue_mutex);
                    this->condition.wait(lock, [this] { return this->stop || !this->tasks.empty(); });
                    if (this->stop && this->tasks.empty())
                        return;
                    active++;
                    task = std::move(this->tasks.front());
                    this->tasks.pop();
                }
                task();
                {
                    std::unique_lock<std::mutex> lock(this->queue_mutex);
                    active--;
                    if (active == 0)
                        condition.notify_all();
                }
            }
        });
    }
}

thread_pool::~thread_pool()
{
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;
    }
    condition.notify_all();
    for (std::thread &worker : workers)
        worker.join();
}

void thread_pool::enqueue(std::function<void()> f)
{
    {
        std::unique_lock<std::mutex> lock(this->queue_mutex);
        this->tasks.push(f);
    }
    this->condition.notify_one();
}

void thread_pool::join()
{
    std::unique_lock<std::mutex> lock(this->queue_mutex);
    if (active != 0 || !this->tasks.empty())
        this->condition.wait(lock, [this] { return active == 0 && this->tasks.empty(); });
    assert(active == 0);
}
}