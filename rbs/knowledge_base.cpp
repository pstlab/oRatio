#include "knowledge_base.h"
#include "fact.h"
#include "node.h"
#include "rule.h"
#include "kb_parser.h"
#include <sstream>
#include <fstream>
#include <cassert>

namespace kb
{
    RBS_EXPORT knowledge_base::knowledge_base() {}
    RBS_EXPORT knowledge_base::~knowledge_base()
    {
        // we delete the facts..
        for ([[maybe_unused]] const auto &[f_name, f] : facts)
            delete f;
        // we delete the predicates..
        for ([[maybe_unused]] const auto &[pred_name, pred] : predicates)
            delete pred;
        // we delete the rules..
        for ([[maybe_unused]] const auto &[r_name, r] : rules)
            delete r;
    }

    RBS_EXPORT void knowledge_base::read(const std::string &script)
    {
        std::stringstream ss(script);
        kb_parser prs(ss);
        ast::compilation_unit *cu = static_cast<ast::compilation_unit *>(prs.parse());
        cus.push_back(cu);

        cu->declare(*this);
        cu->execute(*this, xprs);
    }
    RBS_EXPORT void knowledge_base::read(const std::vector<std::string> &files)
    {
        std::vector<ast::compilation_unit *> c_cus;
        for (const auto &f : files)
            if (std::ifstream ifs(f); ifs)
            {
                kb_parser prs(ifs);
                ast::compilation_unit *cu = static_cast<ast::compilation_unit *>(prs.parse());
                ifs.close();
                c_cus.push_back(cu);
                cus.push_back(cu);
            }
            else
                throw std::invalid_argument("cannot find file '" + f + "'");

        for (const auto &cu : c_cus)
            cu->declare(*this);
        for (const auto &cu : c_cus)
            cu->execute(*this, xprs);
    }

    void knowledge_base::set(const std::string &fn, expr xpr)
    {
        xprs[fn] = xpr;
    }

    predicate &knowledge_base::create_predicate(const std::string &p_name)
    {
        assert(!exists_predicate(p_name));
        predicate *p = new predicate(p_name);
        predicates[p_name] = p;
        return *p;
    }
} // namespace kb
