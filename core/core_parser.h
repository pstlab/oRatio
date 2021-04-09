#pragma once

#include "core_export.h"
#include "riddle_parser.h"

namespace ratio
{
  class scope;
  class context;
  class expr;

  namespace ast
  {
    class expression : public riddle::ast::expression
    {
    public:
      expression();
      expression(const expression &orig) = delete;
      virtual ~expression();

      virtual expr evaluate(const scope &scp, context &ctx) const = 0;
    };

    class bool_literal_expression : public riddle::ast::bool_literal_expression, public expression
    {
    public:
      CORE_EXPORT bool_literal_expression(const riddle::bool_token &l);
      bool_literal_expression(const bool_literal_expression &orig) = delete;
      CORE_EXPORT virtual ~bool_literal_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class int_literal_expression : public riddle::ast::int_literal_expression, public expression
    {
    public:
      CORE_EXPORT int_literal_expression(const riddle::int_token &l);
      int_literal_expression(const int_literal_expression &orig) = delete;
      CORE_EXPORT virtual ~int_literal_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class real_literal_expression : public riddle::ast::real_literal_expression, public expression
    {
    public:
      CORE_EXPORT real_literal_expression(const riddle::real_token &l);
      real_literal_expression(const real_literal_expression &orig) = delete;
      CORE_EXPORT virtual ~real_literal_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class string_literal_expression : public riddle::ast::string_literal_expression, public expression
    {
    public:
      CORE_EXPORT string_literal_expression(const riddle::string_token &l);
      string_literal_expression(const string_literal_expression &orig) = delete;
      CORE_EXPORT virtual ~string_literal_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class cast_expression : public riddle::ast::cast_expression, public expression
    {
    public:
      cast_expression(const std::vector<riddle::id_token> &tp, const riddle::ast::expression *const e);
      cast_expression(const cast_expression &orig) = delete;
      virtual ~cast_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class plus_expression : public riddle::ast::plus_expression, public expression
    {
    public:
      plus_expression(const riddle::ast::expression *const e);
      plus_expression(const plus_expression &orig) = delete;
      virtual ~plus_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class minus_expression : public riddle::ast::minus_expression, public expression
    {
    public:
      minus_expression(const riddle::ast::expression *const e);
      minus_expression(const minus_expression &orig) = delete;
      virtual ~minus_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class not_expression : public riddle::ast::not_expression, public expression
    {
    public:
      not_expression(const riddle::ast::expression *const e);
      not_expression(const not_expression &orig) = delete;
      virtual ~not_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class constructor_expression : public riddle::ast::constructor_expression, public expression
    {
    public:
      constructor_expression(const std::vector<riddle::id_token> &it, const std::vector<const riddle::ast::expression *> &es);
      constructor_expression(const constructor_expression &orig) = delete;
      virtual ~constructor_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class eq_expression : public riddle::ast::eq_expression, public expression
    {
    public:
      eq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r);
      eq_expression(const eq_expression &orig) = delete;
      virtual ~eq_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class neq_expression : public riddle::ast::neq_expression, public expression
    {
    public:
      neq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r);
      neq_expression(const neq_expression &orig) = delete;
      virtual ~neq_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class lt_expression : public riddle::ast::lt_expression, public expression
    {
    public:
      CORE_EXPORT lt_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r);
      lt_expression(const lt_expression &orig) = delete;
      CORE_EXPORT virtual ~lt_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class leq_expression : public riddle::ast::leq_expression, public expression
    {
    public:
      CORE_EXPORT leq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r);
      leq_expression(const leq_expression &orig) = delete;
      CORE_EXPORT virtual ~leq_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class geq_expression : public riddle::ast::geq_expression, public expression
    {
    public:
      CORE_EXPORT geq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r);
      geq_expression(const geq_expression &orig) = delete;
      CORE_EXPORT virtual ~geq_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class gt_expression : public riddle::ast::gt_expression, public expression
    {
    public:
      CORE_EXPORT gt_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r);
      gt_expression(const gt_expression &orig) = delete;
      CORE_EXPORT virtual ~gt_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class function_expression : public riddle::ast::function_expression, public expression
    {
    public:
      function_expression(const std::vector<riddle::id_token> &is, const riddle::id_token &fn, const std::vector<const riddle::ast::expression *> &es);
      function_expression(const function_expression &orig) = delete;
      virtual ~function_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class id_expression : public riddle::ast::id_expression, public expression
    {
    public:
      CORE_EXPORT id_expression(const std::vector<riddle::id_token> &is);
      id_expression(const id_expression &orig) = delete;
      CORE_EXPORT virtual ~id_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class implication_expression : public riddle::ast::implication_expression, public expression
    {
    public:
      implication_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r);
      implication_expression(const implication_expression &orig) = delete;
      virtual ~implication_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class disjunction_expression : public riddle::ast::disjunction_expression, public expression
    {
    public:
      disjunction_expression(const std::vector<const riddle::ast::expression *> &es);
      disjunction_expression(const disjunction_expression &orig) = delete;
      virtual ~disjunction_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class conjunction_expression : public riddle::ast::conjunction_expression, public expression
    {
    public:
      conjunction_expression(const std::vector<const riddle::ast::expression *> &es);
      conjunction_expression(const conjunction_expression &orig) = delete;
      virtual ~conjunction_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class exct_one_expression : public riddle::ast::exct_one_expression, public expression
    {
    public:
      exct_one_expression(const std::vector<const riddle::ast::expression *> &es);
      exct_one_expression(const exct_one_expression &orig) = delete;
      virtual ~exct_one_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class addition_expression : public riddle::ast::addition_expression, public expression
    {
    public:
      addition_expression(const std::vector<const riddle::ast::expression *> &es);
      addition_expression(const addition_expression &orig) = delete;
      virtual ~addition_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class subtraction_expression : public riddle::ast::subtraction_expression, public expression
    {
    public:
      subtraction_expression(const std::vector<const riddle::ast::expression *> &es);
      subtraction_expression(const subtraction_expression &orig) = delete;
      virtual ~subtraction_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class multiplication_expression : public riddle::ast::multiplication_expression, public expression
    {
    public:
      multiplication_expression(const std::vector<const riddle::ast::expression *> &es);
      multiplication_expression(const multiplication_expression &orig) = delete;
      virtual ~multiplication_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class division_expression : public riddle::ast::division_expression, public expression
    {
    public:
      division_expression(const std::vector<const riddle::ast::expression *> &es);
      division_expression(const division_expression &orig) = delete;
      virtual ~division_expression();

      expr evaluate(const scope &scp, context &ctx) const override;
    };

    class statement : public riddle::ast::statement
    {
    public:
      statement();
      statement(const statement &orig) = delete;
      virtual ~statement();

      virtual void execute(const scope &scp, context &ctx) const = 0;
    };

    class local_field_statement : public riddle::ast::local_field_statement, public statement
    {
    public:
      local_field_statement(const std::vector<riddle::id_token> &ft, const std::vector<riddle::id_token> &ns, const std::vector<const riddle::ast::expression *> &es);
      local_field_statement(const local_field_statement &orig) = delete;
      virtual ~local_field_statement();

      void execute(const scope &scp, context &ctx) const override;
    };

    class assignment_statement : public riddle::ast::assignment_statement, public statement
    {
    public:
      assignment_statement(const std::vector<riddle::id_token> &is, const riddle::id_token &i, const riddle::ast::expression *const e);
      assignment_statement(const assignment_statement &orig) = delete;
      virtual ~assignment_statement();

      void execute(const scope &scp, context &ctx) const override;
    };

    class expression_statement : public riddle::ast::expression_statement, public statement
    {
    public:
      CORE_EXPORT expression_statement(const riddle::ast::expression *const e);
      expression_statement(const expression_statement &orig) = delete;
      CORE_EXPORT virtual ~expression_statement();

      void execute(const scope &scp, context &ctx) const override;
    };

    class disjunction_statement : public riddle::ast::disjunction_statement, public statement
    {
    public:
      disjunction_statement(const std::vector<std::pair<const std::vector<const riddle::ast::statement *>, const riddle::ast::expression *const>> &conjs);
      disjunction_statement(const disjunction_statement &orig) = delete;
      virtual ~disjunction_statement();

      void execute(const scope &scp, context &ctx) const override;
    };

    class conjunction_statement : public riddle::ast::conjunction_statement, public statement
    {
    public:
      conjunction_statement(const std::vector<const riddle::ast::statement *> &stmnts);
      conjunction_statement(const conjunction_statement &orig) = delete;
      virtual ~conjunction_statement();

      void execute(const scope &scp, context &ctx) const override;
    };

    class formula_statement : public riddle::ast::formula_statement, public statement
    {
    public:
      formula_statement(const bool &isf, const riddle::id_token &fn, const std::vector<riddle::id_token> &scp, const riddle::id_token &pn, const std::vector<std::pair<const riddle::id_token, const riddle::ast::expression *const>> &assns);
      formula_statement(const formula_statement &orig) = delete;
      virtual ~formula_statement();

      void execute(const scope &scp, context &ctx) const override;
    };

    class return_statement : public riddle::ast::return_statement, public statement
    {
    public:
      return_statement(const riddle::ast::expression *const e);
      return_statement(const return_statement &orig) = delete;
      virtual ~return_statement();

      void execute(const scope &scp, context &ctx) const override;
    };

    class type_declaration : public riddle::ast::type_declaration
    {
    public:
      type_declaration();
      type_declaration(const type_declaration &orig) = delete;
      virtual ~type_declaration();

      virtual void declare(scope &) const {}
      virtual void refine(scope &) const {}
    };

    class method_declaration : public riddle::ast::method_declaration
    {
    public:
      method_declaration(const std::vector<riddle::id_token> &rt, const riddle::id_token &n, const std::vector<std::pair<const std::vector<riddle::id_token>, const riddle::id_token>> &pars, const std::vector<const riddle::ast::statement *> &stmnts);
      method_declaration(const method_declaration &orig) = delete;
      virtual ~method_declaration();

      void refine(scope &scp) const;
    };

    class predicate_declaration : public riddle::ast::predicate_declaration
    {
    public:
      predicate_declaration(const riddle::id_token &n, const std::vector<std::pair<const std::vector<riddle::id_token>, const riddle::id_token>> &pars, const std::vector<std::vector<riddle::id_token>> &pl, const std::vector<const riddle::ast::statement *> &stmnts);
      predicate_declaration(const predicate_declaration &orig) = delete;
      virtual ~predicate_declaration();

      void refine(scope &scp) const;
    };

    class typedef_declaration : public riddle::ast::typedef_declaration, public type_declaration
    {
    public:
      typedef_declaration(const riddle::id_token &n, const riddle::id_token &pt, const riddle::ast::expression *const e);
      typedef_declaration(const typedef_declaration &orig) = delete;
      virtual ~typedef_declaration();

      void declare(scope &scp) const override;
    };

    class enum_declaration : public riddle::ast::enum_declaration, public type_declaration
    {
    public:
      enum_declaration(const riddle::id_token &n, const std::vector<riddle::string_token> &es, const std::vector<std::vector<riddle::id_token>> &trs);
      enum_declaration(const enum_declaration &orig) = delete;
      virtual ~enum_declaration();

      void declare(scope &scp) const override;
      void refine(scope &scp) const override;
    };

    class variable_declaration : public riddle::ast::variable_declaration
    {
      friend class field_declaration;

    public:
      variable_declaration(const riddle::id_token &n, const riddle::ast::expression *const e = nullptr) : riddle::ast::variable_declaration(n, e) {}
      variable_declaration(const variable_declaration &orig) = delete;
      virtual ~variable_declaration() {}
    };

    class field_declaration : public riddle::ast::field_declaration
    {
    public:
      field_declaration(const std::vector<riddle::id_token> &tp, const std::vector<const riddle::ast::variable_declaration *> &ds);
      field_declaration(const field_declaration &orig) = delete;
      virtual ~field_declaration();

      void refine(scope &scp) const;
    };

    class constructor_declaration : public riddle::ast::constructor_declaration
    {
    public:
      constructor_declaration(const std::vector<std::pair<const std::vector<riddle::id_token>, const riddle::id_token>> &pars, const std::vector<std::pair<const riddle::id_token, const std::vector<const riddle::ast::expression *>>> &il, const std::vector<const riddle::ast::statement *> &stmnts);
      constructor_declaration(const constructor_declaration &orig) = delete;
      virtual ~constructor_declaration();

      void refine(scope &scp) const;
    };

    class class_declaration : public riddle::ast::class_declaration, public type_declaration
    {
    public:
      class_declaration(const riddle::id_token &n, const std::vector<std::vector<riddle::id_token>> &bcs, const std::vector<const riddle::ast::field_declaration *> &fs, const std::vector<const riddle::ast::constructor_declaration *> &cs, const std::vector<const riddle::ast::method_declaration *> &ms, const std::vector<const riddle::ast::predicate_declaration *> &ps, const std::vector<const riddle::ast::type_declaration *> &ts);
      class_declaration(const class_declaration &orig) = delete;
      virtual ~class_declaration();

      void declare(scope &scp) const override;
      void refine(scope &scp) const override;
    };

    class compilation_unit : public riddle::ast::compilation_unit
    {
    public:
      compilation_unit(const std::vector<const riddle::ast::method_declaration *> &ms, const std::vector<const riddle::ast::predicate_declaration *> &ps, const std::vector<const riddle::ast::type_declaration *> &ts, const std::vector<const riddle::ast::statement *> &stmnts);
      compilation_unit(const compilation_unit &orig) = delete;
      virtual ~compilation_unit();

      void declare(scope &scp) const;
      void refine(scope &scp) const;
      void execute(const scope &scp, context &ctx) const;
    };
  } // namespace ast

  class riddle_parser : public riddle::parser
  {
  public:
    riddle_parser(std::istream &is);
    riddle_parser(const riddle_parser &orig) = delete;
    virtual ~riddle_parser();

  private:
    /**
   * The declarations.
   */
    ast::method_declaration *new_method_declaration(const std::vector<riddle::id_token> &rt, const riddle::id_token &n, const std::vector<std::pair<const std::vector<riddle::id_token>, const riddle::id_token>> &pars, const std::vector<const riddle::ast::statement *> &stmnts) const noexcept override { return new ast::method_declaration(rt, n, pars, stmnts); }
    ast::predicate_declaration *new_predicate_declaration(const riddle::id_token &n, const std::vector<std::pair<const std::vector<riddle::id_token>, const riddle::id_token>> &pars, const std::vector<std::vector<riddle::id_token>> &pl, const std::vector<const riddle::ast::statement *> &stmnts) const noexcept override { return new ast::predicate_declaration(n, pars, pl, stmnts); }
    ast::typedef_declaration *new_typedef_declaration(const riddle::id_token &n, const riddle::id_token &pt, const riddle::ast::expression *const e) const noexcept override { return new ast::typedef_declaration(n, pt, e); }
    ast::enum_declaration *new_enum_declaration(const riddle::id_token &n, const std::vector<riddle::string_token> &es, const std::vector<std::vector<riddle::id_token>> &trs) const noexcept override { return new ast::enum_declaration(n, es, trs); }
    ast::class_declaration *new_class_declaration(const riddle::id_token &n, const std::vector<std::vector<riddle::id_token>> &bcs, const std::vector<const riddle::ast::field_declaration *> &fs, const std::vector<const riddle::ast::constructor_declaration *> &cs, const std::vector<const riddle::ast::method_declaration *> &ms, const std::vector<const riddle::ast::predicate_declaration *> &ps, const std::vector<const riddle::ast::type_declaration *> &ts) const noexcept override { return new ast::class_declaration(n, bcs, fs, cs, ms, ps, ts); }
    ast::variable_declaration *new_variable_declaration(const riddle::id_token &n, const riddle::ast::expression *const e = nullptr) const noexcept override { return new ast::variable_declaration(n, e); }
    ast::field_declaration *new_field_declaration(const std::vector<riddle::id_token> &tp, const std::vector<const riddle::ast::variable_declaration *> &ds) const noexcept override { return new ast::field_declaration(tp, ds); }
    ast::constructor_declaration *new_constructor_declaration(const std::vector<std::pair<const std::vector<riddle::id_token>, const riddle::id_token>> &pars, const std::vector<std::pair<const riddle::id_token, const std::vector<const riddle::ast::expression *>>> &il, const std::vector<const riddle::ast::statement *> &stmnts) const noexcept override { return new ast::constructor_declaration(pars, il, stmnts); }
    ast::compilation_unit *new_compilation_unit(const std::vector<const riddle::ast::method_declaration *> &ms, const std::vector<const riddle::ast::predicate_declaration *> &ps, const std::vector<const riddle::ast::type_declaration *> &ts, const std::vector<const riddle::ast::statement *> &stmnts) const noexcept override { return new ast::compilation_unit(ms, ps, ts, stmnts); }

    /**
   * The statements.
   */
    ast::local_field_statement *new_local_field_statement(const std::vector<riddle::id_token> &ft, const std::vector<riddle::id_token> &ns, const std::vector<const riddle::ast::expression *> &es) const noexcept override { return new ast::local_field_statement(ft, ns, es); }
    ast::assignment_statement *new_assignment_statement(const std::vector<riddle::id_token> &is, const riddle::id_token &i, const riddle::ast::expression *const e) const noexcept override { return new ast::assignment_statement(is, i, e); }
    ast::expression_statement *new_expression_statement(const riddle::ast::expression *const e) const noexcept override { return new ast::expression_statement(e); }
    ast::disjunction_statement *new_disjunction_statement(const std::vector<std::pair<const std::vector<const riddle::ast::statement *>, const riddle::ast::expression *const>> &conjs) const noexcept override { return new ast::disjunction_statement(conjs); }
    ast::conjunction_statement *new_conjunction_statement(const std::vector<const riddle::ast::statement *> &stmnts) const noexcept override { return new ast::conjunction_statement(stmnts); }
    ast::formula_statement *new_formula_statement(const bool &isf, const riddle::id_token &fn, const std::vector<riddle::id_token> &scp, const riddle::id_token &pn, const std::vector<std::pair<const riddle::id_token, const riddle::ast::expression *const>> &assns) const noexcept override { return new ast::formula_statement(isf, fn, scp, pn, assns); }
    ast::return_statement *new_return_statement(const riddle::ast::expression *const e) const noexcept override { return new ast::return_statement(e); }

    /**
   * The expressions.
   */
    ast::bool_literal_expression *new_bool_literal_expression(const riddle::bool_token &l) const noexcept override { return new ast::bool_literal_expression(l); }
    ast::int_literal_expression *new_int_literal_expression(const riddle::int_token &l) const noexcept override { return new ast::int_literal_expression(l); }
    ast::real_literal_expression *new_real_literal_expression(const riddle::real_token &l) const noexcept override { return new ast::real_literal_expression(l); }
    ast::string_literal_expression *new_string_literal_expression(const riddle::string_token &l) const noexcept override { return new ast::string_literal_expression(l); }
    ast::cast_expression *new_cast_expression(const std::vector<riddle::id_token> &tp, const riddle::ast::expression *const e) const noexcept override { return new ast::cast_expression(tp, e); }
    ast::plus_expression *new_plus_expression(const riddle::ast::expression *const e) const noexcept override { return new ast::plus_expression(e); }
    ast::minus_expression *new_minus_expression(const riddle::ast::expression *const e) const noexcept override { return new ast::minus_expression(e); }
    ast::not_expression *new_not_expression(const riddle::ast::expression *const e) const noexcept override { return new ast::not_expression(e); }
    ast::constructor_expression *new_constructor_expression(const std::vector<riddle::id_token> &it, const std::vector<const riddle::ast::expression *> &es) const noexcept override { return new ast::constructor_expression(it, es); }
    ast::eq_expression *new_eq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) const noexcept override { return new ast::eq_expression(l, r); }
    ast::neq_expression *new_neq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) const noexcept override { return new ast::neq_expression(l, r); }
    ast::lt_expression *new_lt_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) const noexcept override { return new ast::lt_expression(l, r); }
    ast::leq_expression *new_leq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) const noexcept override { return new ast::leq_expression(l, r); }
    ast::geq_expression *new_geq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) const noexcept override { return new ast::geq_expression(l, r); }
    ast::gt_expression *new_gt_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) const noexcept override { return new ast::gt_expression(l, r); }
    ast::function_expression *new_function_expression(const std::vector<riddle::id_token> &is, const riddle::id_token &fn, const std::vector<const riddle::ast::expression *> &es) const noexcept override { return new ast::function_expression(is, fn, es); }
    ast::id_expression *new_id_expression(const std::vector<riddle::id_token> &is) const noexcept override { return new ast::id_expression(is); }
    ast::implication_expression *new_implication_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) const noexcept override { return new ast::implication_expression(l, r); }
    ast::disjunction_expression *new_disjunction_expression(const std::vector<const riddle::ast::expression *> &es) const noexcept override { return new ast::disjunction_expression(es); }
    ast::conjunction_expression *new_conjunction_expression(const std::vector<const riddle::ast::expression *> &es) const noexcept override { return new ast::conjunction_expression(es); }
    ast::exct_one_expression *new_exct_one_expression(const std::vector<const riddle::ast::expression *> &es) const noexcept override { return new ast::exct_one_expression(es); }
    ast::addition_expression *new_addition_expression(const std::vector<const riddle::ast::expression *> &es) const noexcept override { return new ast::addition_expression(es); }
    ast::subtraction_expression *new_subtraction_expression(const std::vector<const riddle::ast::expression *> &es) const noexcept override { return new ast::subtraction_expression(es); }
    ast::multiplication_expression *new_multiplication_expression(const std::vector<const riddle::ast::expression *> &es) const noexcept override { return new ast::multiplication_expression(es); }
    ast::division_expression *new_division_expression(const std::vector<const riddle::ast::expression *> &es) const noexcept override { return new ast::division_expression(es); }
  };
} // namespace ratio