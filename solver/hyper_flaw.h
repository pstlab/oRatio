#pragma once

#include "flaw.h"
#include "resolver.h"

namespace ratio
{

class hyper_flaw : public flaw
{
    friend class solver;

  public:
    hyper_flaw(solver &slv, resolver *const cause, const std::vector<flaw *> &fs);
    hyper_flaw(const hyper_flaw &orig) = delete;
    virtual ~hyper_flaw();

    std::string get_label() const override
    {
        std::string f_str = "φ" + std::to_string(get_phi()) + " {";
        for (std::vector<flaw *>::const_iterator f_it = flaws.cbegin(); f_it != flaws.cend(); ++f_it)
        {
            if (f_it != flaws.cbegin())
                f_str += ", ";
            f_str += (*f_it)->get_label();
        }
        f_str += "}";
        return f_str;
    }

  private:
    void compute_resolvers() override;

    class hyper_resolver : public resolver
    {
      public:
        hyper_resolver(solver &slv, hyper_flaw &s_flaw, const smt::var &app_r, const smt::rational &cst, const std::vector<resolver *> &rs);
        hyper_resolver(const hyper_resolver &that) = delete;
        virtual ~hyper_resolver();

        std::string get_label() const override
        {
            std::string r_str = "ρ" + std::to_string(rho) + " {";
            for (std::vector<resolver *>::const_iterator r_it = resolvers.cbegin(); r_it != resolvers.cend(); ++r_it)
            {
                if (r_it != resolvers.cbegin())
                    r_str += ", ";
                r_str += (*r_it)->get_label();
            }
            r_str += "}";
            return r_str;
        }

      private:
        void apply() override;

      private:
        const std::vector<resolver *> resolvers;
    };

  private:
    const std::vector<flaw *> flaws;
};
}