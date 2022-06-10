#pragma once

#include "rbs_parser.h"
#include "kb_item.h"
#include <unordered_map>

namespace kb
{
  class knowledge_base;

  namespace ast
  {
    class expression : public rbs::ast::expression
    {
    public:
      expression() = default;
      expression(const expression &orig) = delete;
      virtual ~expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const = 0;
    };

    class bool_literal_expression : public rbs::ast::bool_literal_expression, public expression
    {
    public:
      RBS_EXPORT bool_literal_expression(const rbs::bool_token &l);
      bool_literal_expression(const bool_literal_expression &orig) = delete;
      RBS_EXPORT virtual ~bool_literal_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class int_literal_expression : public rbs::ast::int_literal_expression, public expression
    {
    public:
      RBS_EXPORT int_literal_expression(const rbs::int_token &l);
      int_literal_expression(const int_literal_expression &orig) = delete;
      RBS_EXPORT virtual ~int_literal_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class real_literal_expression : public rbs::ast::real_literal_expression, public expression
    {
    public:
      RBS_EXPORT real_literal_expression(const rbs::real_token &l);
      real_literal_expression(const real_literal_expression &orig) = delete;
      RBS_EXPORT virtual ~real_literal_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class string_literal_expression : public rbs::ast::string_literal_expression, public expression
    {
    public:
      RBS_EXPORT string_literal_expression(const rbs::string_token &l);
      string_literal_expression(const string_literal_expression &orig) = delete;
      RBS_EXPORT virtual ~string_literal_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class plus_expression : public rbs::ast::plus_expression, public expression
    {
    public:
      plus_expression(const rbs::ast::expression *const e);
      plus_expression(const plus_expression &orig) = delete;
      virtual ~plus_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class minus_expression : public rbs::ast::minus_expression, public expression
    {
    public:
      minus_expression(const rbs::ast::expression *const e);
      minus_expression(const minus_expression &orig) = delete;
      virtual ~minus_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class not_expression : public rbs::ast::not_expression, public expression
    {
    public:
      not_expression(const rbs::ast::expression *const e);
      not_expression(const not_expression &orig) = delete;
      virtual ~not_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class eq_expression : public rbs::ast::eq_expression, public expression
    {
    public:
      eq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r);
      eq_expression(const eq_expression &orig) = delete;
      virtual ~eq_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class neq_expression : public rbs::ast::neq_expression, public expression
    {
    public:
      neq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r);
      neq_expression(const neq_expression &orig) = delete;
      virtual ~neq_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class lt_expression : public rbs::ast::lt_expression, public expression
    {
    public:
      RBS_EXPORT lt_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r);
      lt_expression(const lt_expression &orig) = delete;
      RBS_EXPORT virtual ~lt_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class leq_expression : public rbs::ast::leq_expression, public expression
    {
    public:
      RBS_EXPORT leq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r);
      leq_expression(const leq_expression &orig) = delete;
      RBS_EXPORT virtual ~leq_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class geq_expression : public rbs::ast::geq_expression, public expression
    {
    public:
      RBS_EXPORT geq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r);
      geq_expression(const geq_expression &orig) = delete;
      RBS_EXPORT virtual ~geq_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class gt_expression : public rbs::ast::gt_expression, public expression
    {
    public:
      RBS_EXPORT gt_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r);
      gt_expression(const gt_expression &orig) = delete;
      RBS_EXPORT virtual ~gt_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class id_expression : public rbs::ast::id_expression, public expression
    {
    public:
      RBS_EXPORT id_expression(const std::vector<rbs::id_token> &is);
      id_expression(const id_expression &orig) = delete;
      RBS_EXPORT virtual ~id_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class implication_expression : public rbs::ast::implication_expression, public expression
    {
    public:
      implication_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r);
      implication_expression(const implication_expression &orig) = delete;
      virtual ~implication_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class disjunction_expression : public rbs::ast::disjunction_expression, public expression
    {
    public:
      disjunction_expression(const std::vector<const rbs::ast::expression *> &es);
      disjunction_expression(const disjunction_expression &orig) = delete;
      virtual ~disjunction_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class conjunction_expression : public rbs::ast::conjunction_expression, public expression
    {
    public:
      conjunction_expression(const std::vector<const rbs::ast::expression *> &es);
      conjunction_expression(const conjunction_expression &orig) = delete;
      virtual ~conjunction_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class exct_one_expression : public rbs::ast::exct_one_expression, public expression
    {
    public:
      exct_one_expression(const std::vector<const rbs::ast::expression *> &es);
      exct_one_expression(const exct_one_expression &orig) = delete;
      virtual ~exct_one_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class addition_expression : public rbs::ast::addition_expression, public expression
    {
    public:
      addition_expression(const std::vector<const rbs::ast::expression *> &es);
      addition_expression(const addition_expression &orig) = delete;
      virtual ~addition_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class subtraction_expression : public rbs::ast::subtraction_expression, public expression
    {
    public:
      subtraction_expression(const std::vector<const rbs::ast::expression *> &es);
      subtraction_expression(const subtraction_expression &orig) = delete;
      virtual ~subtraction_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class multiplication_expression : public rbs::ast::multiplication_expression, public expression
    {
    public:
      multiplication_expression(const std::vector<const rbs::ast::expression *> &es);
      multiplication_expression(const multiplication_expression &orig) = delete;
      virtual ~multiplication_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class division_expression : public rbs::ast::division_expression, public expression
    {
    public:
      division_expression(const std::vector<const rbs::ast::expression *> &es);
      division_expression(const division_expression &orig) = delete;
      virtual ~division_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class fact_expression : public rbs::ast::fact_expression, public expression
    {
    public:
      fact_expression(const rbs::id_token &fn, const rbs::id_token &pn, const std::vector<std::pair<const rbs::id_token, const rbs::ast::expression *const>> &assns);
      fact_expression(const fact_expression &orig) = delete;
      virtual ~fact_expression() = default;

      virtual std::vector<std::vector<expr>> evaluate(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class statement : public rbs::ast::statement
    {
    public:
      statement() = default;
      statement(const statement &orig) = delete;
      virtual ~statement() = default;

      virtual void execute(std::unordered_map<std::string, expr> &ctx) const = 0;
    };

    class assert_statement : public rbs::ast::assert_statement, public statement
    {
    public:
      assert_statement(const rbs::id_token &fn, const rbs::id_token &pn, const std::vector<std::pair<const rbs::id_token, const rbs::ast::expression *const>> &assns);
      assert_statement(const assert_statement &orig) = delete;
      virtual ~assert_statement() = default;

      virtual void execute(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class retract_statement : public rbs::ast::retract_statement, public statement
    {
    public:
      retract_statement(const std::vector<rbs::id_token> &fns);
      retract_statement(const retract_statement &orig) = delete;
      virtual ~retract_statement() = default;

      virtual void execute(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class assignment_statement : public rbs::ast::assignment_statement, public statement
    {
    public:
      assignment_statement(const std::vector<std::vector<rbs::id_token>> &is, const std::vector<const rbs::ast::expression *> es);
      assignment_statement(const assignment_statement &orig) = delete;
      virtual ~assignment_statement() = default;

      virtual void execute(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class function_statement : public rbs::ast::function_statement, public statement
    {
    public:
      function_statement(const rbs::id_token &fn, const std::vector<const rbs::ast::expression *> &xprs);
      function_statement(const function_statement &orig) = delete;
      virtual ~function_statement() = default;

      virtual void execute(std::unordered_map<std::string, expr> &ctx) const override;
    };

    class rule_declaration : public rbs::ast::rule_declaration
    {
    public:
      rule_declaration(const rbs::id_token &n, const rbs::ast::expression *const cond, const std::vector<const rbs::ast::statement *> &ss);
      rule_declaration(const rule_declaration &orig) = delete;
      virtual ~rule_declaration() = default;

      virtual void declare(std::unordered_map<std::string, expr> &ctx) const;
    };

    class compilation_unit : public rbs::ast::compilation_unit
    {
    public:
      compilation_unit(const std::vector<const rbs::ast::statement *> &ss, const std::vector<const rbs::ast::rule_declaration *> &rs);
      compilation_unit(const compilation_unit &orig) = delete;
      virtual ~compilation_unit() = default;

      virtual void declare(std::unordered_map<std::string, expr> &ctx) const;
    };
  } // namespace ast

  class kb_parser : public rbs::parser
  {
  public:
    kb_parser(std::istream &is);
    kb_parser(const kb_parser &orig) = delete;
    virtual ~kb_parser() = default;

  private:
    /**
     * The declarations.
     */
    ast::rule_declaration *new_rule_declaration(const rbs::id_token &n, const rbs::ast::expression *const cond, const std::vector<const rbs::ast::statement *> &ss) const noexcept override { return new ast::rule_declaration(n, cond, ss); }
    ast::compilation_unit *new_compilation_unit(const std::vector<const rbs::ast::statement *> &fs, const std::vector<const rbs::ast::rule_declaration *> &rs) const noexcept override { return new ast::compilation_unit(fs, rs); }

    /**
     * The statements.
     */
    ast::assert_statement *new_assert_statement(const rbs::id_token &fn, const rbs::id_token &pn, const std::vector<std::pair<const rbs::id_token, const rbs::ast::expression *const>> &assns) const noexcept override { return new ast::assert_statement(fn, pn, assns); }
    ast::retract_statement *new_retract_statement(const std::vector<rbs::id_token> &fns) const noexcept override { return new ast::retract_statement(fns); }
    ast::assignment_statement *new_assignment_statement(const std::vector<std::vector<rbs::id_token>> &is, const std::vector<const rbs::ast::expression *> es) const noexcept override { return new ast::assignment_statement(is, es); }
    ast::function_statement *new_function_statement(const rbs::id_token &fn, const std::vector<const rbs::ast::expression *> &xprs) const noexcept override { return new ast::function_statement(fn, xprs); }

    /**
     * The expressions.
     */
    ast::bool_literal_expression *new_bool_literal_expression(const rbs::bool_token &l) const noexcept override { return new ast::bool_literal_expression(l); }
    ast::int_literal_expression *new_int_literal_expression(const rbs::int_token &l) const noexcept override { return new ast::int_literal_expression(l); }
    ast::real_literal_expression *new_real_literal_expression(const rbs::real_token &l) const noexcept override { return new ast::real_literal_expression(l); }
    ast::string_literal_expression *new_string_literal_expression(const rbs::string_token &l) const noexcept override { return new ast::string_literal_expression(l); }
    ast::plus_expression *new_plus_expression(const rbs::ast::expression *const e) const noexcept override { return new ast::plus_expression(e); }
    ast::minus_expression *new_minus_expression(const rbs::ast::expression *const e) const noexcept override { return new ast::minus_expression(e); }
    ast::not_expression *new_not_expression(const rbs::ast::expression *const e) const noexcept override { return new ast::not_expression(e); }
    ast::eq_expression *new_eq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) const noexcept override { return new ast::eq_expression(l, r); }
    ast::neq_expression *new_neq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) const noexcept override { return new ast::neq_expression(l, r); }
    ast::lt_expression *new_lt_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) const noexcept override { return new ast::lt_expression(l, r); }
    ast::leq_expression *new_leq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) const noexcept override { return new ast::leq_expression(l, r); }
    ast::geq_expression *new_geq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) const noexcept override { return new ast::geq_expression(l, r); }
    ast::gt_expression *new_gt_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) const noexcept override { return new ast::gt_expression(l, r); }
    ast::id_expression *new_id_expression(const std::vector<rbs::id_token> &is) const noexcept override { return new ast::id_expression(is); }
    ast::implication_expression *new_implication_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) const noexcept override { return new ast::implication_expression(l, r); }
    ast::disjunction_expression *new_disjunction_expression(const std::vector<const rbs::ast::expression *> &es) const noexcept override { return new ast::disjunction_expression(es); }
    ast::conjunction_expression *new_conjunction_expression(const std::vector<const rbs::ast::expression *> &es) const noexcept override { return new ast::conjunction_expression(es); }
    ast::exct_one_expression *new_exct_one_expression(const std::vector<const rbs::ast::expression *> &es) const noexcept override { return new ast::exct_one_expression(es); }
    ast::addition_expression *new_addition_expression(const std::vector<const rbs::ast::expression *> &es) const noexcept override { return new ast::addition_expression(es); }
    ast::subtraction_expression *new_subtraction_expression(const std::vector<const rbs::ast::expression *> &es) const noexcept override { return new ast::subtraction_expression(es); }
    ast::multiplication_expression *new_multiplication_expression(const std::vector<const rbs::ast::expression *> &es) const noexcept override { return new ast::multiplication_expression(es); }
    ast::division_expression *new_division_expression(const std::vector<const rbs::ast::expression *> &es) const noexcept override { return new ast::division_expression(es); }
    ast::fact_expression *new_fact_expression(const rbs::id_token &fn, const rbs::id_token &pn, const std::vector<std::pair<const rbs::id_token, const rbs::ast::expression *const>> &assns) const noexcept override { return new ast::fact_expression(fn, pn, assns); }
  };
} // namespace kb
