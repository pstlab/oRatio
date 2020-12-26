#include "drl.h"

namespace drl
{
    reply_buffer::reply_buffer(const size_t &size) : size(size) {}
    reply_buffer::~reply_buffer() {}

    void reply_buffer::add(const transition &tr)
    {
        if (storage.size() == size)
        {
            storage[ptr] = tr;
            ptr = (ptr + 1) % size;
        }
        else
            storage.push_back(tr);
    }

    transition_batch reply_buffer::sample(const size_t &batch_size) const
    {
        std::vector<std::vector<double>> states;
        states.reserve(batch_size);
        std::vector<std::vector<double>> next_states;
        next_states.reserve(batch_size);
        std::vector<std::vector<double>> actions;
        actions.reserve(batch_size);
        std::vector<double> rewards;
        rewards.reserve(batch_size);

        const auto rnd_ids = torch::randint(size, batch_size).detach();
        size_t *ptr = reinterpret_cast<size_t *>(rnd_ids.data_ptr());
        for (size_t i = 0; i < batch_size; ++i)
        {
            states.push_back(storage.at(*ptr).state);
            next_states.push_back(storage.at(*ptr).next_state);
            actions.push_back(storage.at(*ptr).action);
            rewards.push_back(storage.at(*ptr).reward);
            ptr++;
        }
        return transition_batch(states, next_states, actions, rewards);
    }
} // namespace drl
