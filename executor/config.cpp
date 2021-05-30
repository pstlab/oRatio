#include "config.h"
#include "solver.h"
#include "predicate.h"

namespace ratio
{
    predicate &get_predicate(const solver &slv, const std::string &pred)
    {
        std::vector<std::string> ids;
        size_t start = 0, end = 0;
        do
        {
            end = pred.find('.', start);
            if (end == std::string::npos)
                end = pred.length();
            std::string token = pred.substr(start, end - start);
            if (!token.empty())
                ids.push_back(token);
            start = end + 1;
        } while (end < pred.length() && start < pred.length());

        if (ids.size() == 1)
            return slv.get_predicate(ids[0]);
        else
        {
            type *tp = &slv.get_type(ids[0]);
            for (size_t i = 1; i < ids.size(); ++i)
                if (i == ids.size() - 1)
                    return tp->get_predicate(ids[i]);
                else
                    tp = &tp->get_type(ids[i]);
        }
        // not found
        throw std::out_of_range(pred);
    }

    EXECUTOR_EXPORT config::config() {}
    EXECUTOR_EXPORT config::~config() {}

    EXECUTOR_EXPORT void config::read(const solver &slv, const std::string &cnfg) { read(slv, std::stringstream(cnfg)); }
    EXECUTOR_EXPORT void config::read(const solver &slv, std::istream &is)
    {
        const auto cnfg = smt::json::from_json(is);

        notify_start.clear();
        if (cnfg->has("notify-start"))
        {
            const smt::array_val &notify_s = static_cast<smt::array_val &>(*cnfg->get("notify-start"));
            for (size_t i = 0; i < notify_s.size(); ++i)
            {
                const smt::string_val &pred = static_cast<smt::string_val &>(*notify_s.get(i));
                notify_start.insert(&get_predicate(slv, pred.get()));
            }
        }

        if (cnfg->has("notify-end"))
        {
            notify_end.clear();
            const smt::array_val &notify_e = static_cast<smt::array_val &>(*cnfg->get("notify-end"));
            for (size_t i = 0; i < notify_e.size(); ++i)
            {
                const smt::string_val &pred = static_cast<smt::string_val &>(*notify_e.get(i));
                notify_end.insert(&get_predicate(slv, pred.get()));
            }
        }

        if (cnfg->has("auto-start"))
        {
            auto_start.clear();
            const smt::array_val &auto_s = static_cast<smt::array_val &>(*cnfg->get("auto-start"));
            for (size_t i = 0; i < auto_s.size(); ++i)
            {
                const smt::string_val &pred = static_cast<smt::string_val &>(*auto_s.get(i));
                auto_start.insert(&get_predicate(slv, pred.get()));
            }
        }

        if (cnfg->has("auto-end"))
        {
            auto_end.clear();
            const smt::array_val &auto_e = static_cast<smt::array_val &>(*cnfg->get("auto-end"));
            for (size_t i = 0; i < auto_e.size(); ++i)
            {
                const smt::string_val &pred = static_cast<smt::string_val &>(*auto_e.get(i));
                auto_end.insert(&get_predicate(slv, pred.get()));
            }
        }
    }
} // namespace ratio
