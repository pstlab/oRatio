#pragma once

#include "rbs_lexer.h"
#include <vector>

namespace rbs
{
  namespace ast
  {
    class expression
    {
    public:
      expression() = default;
      expression(const expression &orig) = delete;
      virtual ~expression() = default;
    };

    class bool_literal_expression : public expression
    {
    public:
      RBS_EXPORT bool_literal_expression(const bool_token &l) : literal(l) {}
      bool_literal_expression(const bool_literal_expression &orig) = delete;
      RBS_EXPORT virtual ~bool_literal_expression() = default;

    protected:
      const bool_token literal;
    };

    class int_literal_expression : public expression
    {
    public:
      RBS_EXPORT int_literal_expression(const int_token &l) : literal(l) {}
      int_literal_expression(const int_literal_expression &orig) = delete;
      RBS_EXPORT virtual ~int_literal_expression() = default;

    protected:
      const int_token literal;
    };

    class real_literal_expression : public expression
    {
    public:
      RBS_EXPORT real_literal_expression(const real_token &l) : literal(l) {}
      real_literal_expression(const real_literal_expression &orig) = delete;
      RBS_EXPORT virtual ~real_literal_expression() = default;

    protected:
      const real_token literal;
    };

    class string_literal_expression : public expression
    {
    public:
      RBS_EXPORT string_literal_expression(const string_token &l) : literal(l) {}
      string_literal_expression(const string_literal_expression &orig) = delete;
      RBS_EXPORT virtual ~string_literal_expression() = default;

    protected:
      const string_token literal;
    };

    class plus_expression : public expression
    {
    public:
      plus_expression(const expression *const e) : xpr(e) {}
      plus_expression(const plus_expression &orig) = delete;
      virtual ~plus_expression() { delete xpr; }

    protected:
      const expression *const xpr;
    };

    class minus_expression : public expression
    {
    public:
      minus_expression(const expression *const e) : xpr(e) {}
      minus_expression(const minus_expression &orig) = delete;
      virtual ~minus_expression() { delete xpr; }

    protected:
      const expression *const xpr;
    };

    class not_expression : public expression
    {
    public:
      not_expression(const expression *const e) : xpr(e) {}
      not_expression(const not_expression &orig) = delete;
      virtual ~not_expression() { delete xpr; }

    protected:
      const expression *const xpr;
    };

    class eq_expression : public expression
    {
    public:
      eq_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
      eq_expression(const eq_expression &orig) = delete;
      virtual ~eq_expression()
      {
        delete left;
        delete right;
      }

    protected:
      const expression *const left;
      const expression *const right;
    };

    class neq_expression : public expression
    {
    public:
      neq_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
      neq_expression(const neq_expression &orig) = delete;
      virtual ~neq_expression()
      {
        delete left;
        delete right;
      }

    protected:
      const expression *const left;
      const expression *const right;
    };

    class lt_expression : public expression
    {
    public:
      RBS_EXPORT lt_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
      lt_expression(const lt_expression &orig) = delete;
      RBS_EXPORT virtual ~lt_expression()
      {
        delete left;
        delete right;
      }

    protected:
      const expression *const left;
      const expression *const right;
    };

    class leq_expression : public expression
    {
    public:
      RBS_EXPORT leq_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
      leq_expression(const leq_expression &orig) = delete;
      RBS_EXPORT virtual ~leq_expression()
      {
        delete left;
        delete right;
      }

    protected:
      const expression *const left;
      const expression *const right;
    };

    class geq_expression : public expression
    {
    public:
      RBS_EXPORT geq_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
      geq_expression(const geq_expression &orig) = delete;
      RBS_EXPORT virtual ~geq_expression()
      {
        delete left;
        delete right;
      }

    protected:
      const expression *const left;
      const expression *const right;
    };

    class gt_expression : public expression
    {
    public:
      RBS_EXPORT gt_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
      gt_expression(const gt_expression &orig) = delete;
      RBS_EXPORT virtual ~gt_expression()
      {
        delete left;
        delete right;
      }

    protected:
      const expression *const left;
      const expression *const right;
    };

    class id_expression : public expression
    {
    public:
      RBS_EXPORT id_expression(const std::vector<id_token> &is) : ids(is) {}
      id_expression(const id_expression &orig) = delete;
      RBS_EXPORT virtual ~id_expression() = default;

    protected:
      const std::vector<id_token> ids;
    };

    class implication_expression : public expression
    {
    public:
      implication_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
      implication_expression(const implication_expression &orig) = delete;
      virtual ~implication_expression()
      {
        delete left;
        delete right;
      }

    protected:
      const expression *const left;
      const expression *const right;
    };

    class disjunction_expression : public expression
    {
    public:
      disjunction_expression(const std::vector<const expression *> &es) : expressions(es) {}
      disjunction_expression(const disjunction_expression &orig) = delete;
      virtual ~disjunction_expression()
      {
        for (const auto &e : expressions)
          delete e;
      }

    protected:
      const std::vector<const expression *> expressions;
    };

    class conjunction_expression : public expression
    {
    public:
      conjunction_expression(const std::vector<const expression *> &es) : expressions(es) {}
      conjunction_expression(const conjunction_expression &orig) = delete;
      virtual ~conjunction_expression()
      {
        for (const auto &e : expressions)
          delete e;
      }

    protected:
      const std::vector<const expression *> expressions;
    };

    class exct_one_expression : public expression
    {
    public:
      exct_one_expression(const std::vector<const expression *> &es) : expressions(es) {}
      exct_one_expression(const exct_one_expression &orig) = delete;
      virtual ~exct_one_expression()
      {
        for (const auto &e : expressions)
          delete e;
      }

    protected:
      const std::vector<const expression *> expressions;
    };

    class addition_expression : public expression
    {
    public:
      addition_expression(const std::vector<const expression *> &es) : expressions(es) {}
      addition_expression(const addition_expression &orig) = delete;
      virtual ~addition_expression()
      {
        for (const auto &e : expressions)
          delete e;
      }

    protected:
      const std::vector<const expression *> expressions;
    };

    class subtraction_expression : public expression
    {
    public:
      subtraction_expression(const std::vector<const expression *> &es) : expressions(es) {}
      subtraction_expression(const subtraction_expression &orig) = delete;
      virtual ~subtraction_expression()
      {
        for (const auto &e : expressions)
          delete e;
      }

    protected:
      const std::vector<const expression *> expressions;
    };

    class multiplication_expression : public expression
    {
    public:
      multiplication_expression(const std::vector<const expression *> &es) : expressions(es) {}
      multiplication_expression(const multiplication_expression &orig) = delete;
      virtual ~multiplication_expression()
      {
        for (const auto &e : expressions)
          delete e;
      }

    protected:
      const std::vector<const expression *> expressions;
    };

    class division_expression : public expression
    {
    public:
      division_expression(const std::vector<const expression *> &es) : expressions(es) {}
      division_expression(const division_expression &orig) = delete;
      virtual ~division_expression()
      {
        for (const auto &e : expressions)
          delete e;
      }

    protected:
      const std::vector<const expression *> expressions;
    };

    class fact_expression : public expression
    {
    public:
      fact_expression(const id_token &fn, const id_token &pn, const std::vector<std::pair<const id_token, const expression *const>> &assns) : fact_name(fn), predicate_name(pn), assignments(assns) {}
      fact_expression(const fact_expression &orig) = delete;
      virtual ~fact_expression()
      {
        for ([[maybe_unused]] const auto &[id_tkn, xpr] : assignments)
          delete xpr;
      }

    protected:
      const id_token fact_name;
      const id_token predicate_name;
      const std::vector<std::pair<const id_token, const expression *const>> assignments;
    };

    class statement
    {
    public:
      statement() = default;
      statement(const statement &orig) = delete;
      virtual ~statement() = default;
    };

    class assert_statement : public statement
    {
    public:
      assert_statement(const id_token &fn, const id_token &pn, const std::vector<std::pair<const id_token, const expression *const>> &assns) : fact_name(fn), predicate_name(pn), assignments(assns) {}
      assert_statement(const assert_statement &orig) = delete;
      virtual ~assert_statement()
      {
        for ([[maybe_unused]] const auto &[id_tkn, xpr] : assignments)
          delete xpr;
      }

    protected:
      const id_token fact_name;
      const id_token predicate_name;
      const std::vector<std::pair<const id_token, const expression *const>> assignments;
    };

    class retract_statement : public statement
    {
    public:
      retract_statement(const std::vector<id_token> &fns) : fact_names(fns) {}
      retract_statement(const retract_statement &orig) = delete;
      virtual ~retract_statement() {}

    protected:
      const std::vector<id_token> fact_names;
    };

    class assignment_statement : public statement
    {
    public:
      assignment_statement(const std::vector<std::vector<id_token>> &is, const std::vector<const expression *> &es) : ids(is), xprs(es) {}
      assignment_statement(const assignment_statement &orig) = delete;
      virtual ~assignment_statement()
      {
        for (const auto &xpr : xprs)
          delete xpr;
      }

    protected:
      const std::vector<std::vector<id_token>> ids;
      const std::vector<const expression *> xprs;
    };

    class function_statement : public statement
    {
    public:
      function_statement(const id_token &fn, const std::vector<const expression *> &xprs) : function_name(fn), expressions(xprs) {}
      function_statement(const function_statement &orig) = delete;
      virtual ~function_statement()
      {
        for (const auto &xpr : expressions)
          delete xpr;
      }

    protected:
      const id_token function_name;
      const std::vector<const expression *> expressions;
    };

    class rule_declaration
    {
    public:
      rule_declaration(const id_token &n, const expression *const cond, const std::vector<const statement *> &ss) : name(n), condition(cond), statements(ss) {}
      rule_declaration(const rule_declaration &orig) = delete;
      virtual ~rule_declaration()
      {
        delete condition;
        for (const auto &stmnt : statements)
          delete stmnt;
      }

    protected:
      const id_token name;
      const expression *const condition;
      const std::vector<const statement *> statements;
    };

    class compilation_unit
    {
    public:
      compilation_unit(const std::vector<const statement *> &ss, const std::vector<const rule_declaration *> &rs) : statements(ss), rules(rs) {}
      compilation_unit(const compilation_unit &orig) = delete;
      virtual ~compilation_unit()
      {
        for (const auto &stmnt : statements)
          delete stmnt;
        for (const auto &r : rules)
          delete r;
      }

    protected:
      const std::vector<const statement *> statements;
      const std::vector<const rule_declaration *> rules;
    };
  } // namespace ast

  class parser
  {
  public:
    RBS_EXPORT parser(std::istream &is);
    parser(const parser &orig) = delete;
    RBS_EXPORT virtual ~parser();

    RBS_EXPORT ast::compilation_unit *parse();

  private:
    token *next();
    bool match(const symbol &sym);
    void backtrack(const size_t &p) noexcept;

    ast::rule_declaration *_rule_declaration();
    ast::statement *_statement();
    ast::expression *_expression(const size_t &pr = 0);

    void error(const std::string &err);

    /**
     * The declarations.
     */
    virtual ast::rule_declaration *new_rule_declaration(const id_token &n, const ast::expression *const cond, const std::vector<const ast::statement *> &ss) const noexcept { return new ast::rule_declaration(n, cond, ss); }
    virtual ast::compilation_unit *new_compilation_unit(const std::vector<const ast::statement *> &fs, const std::vector<const ast::rule_declaration *> &rs) const noexcept { return new ast::compilation_unit(fs, rs); }

    /**
     * The statements.
     */
    virtual ast::assert_statement *new_assert_statement(const id_token &fn, const id_token &pn, const std::vector<std::pair<const id_token, const ast::expression *const>> &assns) const noexcept { return new ast::assert_statement(fn, pn, assns); }
    virtual ast::retract_statement *new_retract_statement(const std::vector<id_token> &fns) const noexcept { return new ast::retract_statement(fns); }
    virtual ast::assignment_statement *new_assignment_statement(const std::vector<std::vector<id_token>> &is, const std::vector<const ast::expression *> es) const noexcept { return new ast::assignment_statement(is, es); }
    virtual ast::function_statement *new_function_statement(const id_token &fn, const std::vector<const ast::expression *> &xprs) const noexcept { return new ast::function_statement(fn, xprs); }

    /**
     * The expressions.
     */
    virtual ast::bool_literal_expression *new_bool_literal_expression(const bool_token &l) const noexcept { return new ast::bool_literal_expression(l); }
    virtual ast::int_literal_expression *new_int_literal_expression(const int_token &l) const noexcept { return new ast::int_literal_expression(l); }
    virtual ast::real_literal_expression *new_real_literal_expression(const real_token &l) const noexcept { return new ast::real_literal_expression(l); }
    virtual ast::string_literal_expression *new_string_literal_expression(const string_token &l) const noexcept { return new ast::string_literal_expression(l); }
    virtual ast::plus_expression *new_plus_expression(const ast::expression *const e) const noexcept { return new ast::plus_expression(e); }
    virtual ast::minus_expression *new_minus_expression(const ast::expression *const e) const noexcept { return new ast::minus_expression(e); }
    virtual ast::not_expression *new_not_expression(const ast::expression *const e) const noexcept { return new ast::not_expression(e); }
    virtual ast::eq_expression *new_eq_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::eq_expression(l, r); }
    virtual ast::neq_expression *new_neq_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::neq_expression(l, r); }
    virtual ast::lt_expression *new_lt_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::lt_expression(l, r); }
    virtual ast::leq_expression *new_leq_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::leq_expression(l, r); }
    virtual ast::geq_expression *new_geq_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::geq_expression(l, r); }
    virtual ast::gt_expression *new_gt_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::gt_expression(l, r); }
    virtual ast::id_expression *new_id_expression(const std::vector<id_token> &is) const noexcept { return new ast::id_expression(is); }
    virtual ast::implication_expression *new_implication_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::implication_expression(l, r); }
    virtual ast::disjunction_expression *new_disjunction_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::disjunction_expression(es); }
    virtual ast::conjunction_expression *new_conjunction_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::conjunction_expression(es); }
    virtual ast::exct_one_expression *new_exct_one_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::exct_one_expression(es); }
    virtual ast::addition_expression *new_addition_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::addition_expression(es); }
    virtual ast::subtraction_expression *new_subtraction_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::subtraction_expression(es); }
    virtual ast::multiplication_expression *new_multiplication_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::multiplication_expression(es); }
    virtual ast::division_expression *new_division_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::division_expression(es); }
    virtual ast::fact_expression *new_fact_expression(const id_token &fn, const id_token &pn, const std::vector<std::pair<const id_token, const ast::expression *const>> &assns) const noexcept { return new ast::fact_expression(fn, pn, assns); }

  private:
    lexer lex;                // the current lexer..
    token *tk = nullptr;      // the current lookahead token..
    std::vector<token *> tks; // all the tokens parsed so far..
    size_t pos = 0;           // the current position within tks'..
  };
} // namespace rbs
