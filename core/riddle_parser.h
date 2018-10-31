#pragma once

#include "parser.h"

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
  bool_literal_expression(const bool &l);
  bool_literal_expression(const bool_literal_expression &orig) = delete;
  virtual ~bool_literal_expression();

  expr evaluate(const scope &scp, context &ctx) const override;
};

class int_literal_expression : public riddle::ast::int_literal_expression, public expression
{
public:
  int_literal_expression(const smt::I &l);
  int_literal_expression(const int_literal_expression &orig) = delete;
  virtual ~int_literal_expression();

  expr evaluate(const scope &scp, context &ctx) const override;
};

class real_literal_expression : public riddle::ast::real_literal_expression, public expression
{
public:
  real_literal_expression(const smt::rational &l);
  real_literal_expression(const real_literal_expression &orig) = delete;
  virtual ~real_literal_expression();

  expr evaluate(const scope &scp, context &ctx) const override;
};

class string_literal_expression : public riddle::ast::string_literal_expression, public expression
{
public:
  string_literal_expression(const std::string &l);
  string_literal_expression(const string_literal_expression &orig) = delete;
  virtual ~string_literal_expression();

  expr evaluate(const scope &scp, context &ctx) const override;
};

class cast_expression : public riddle::ast::cast_expression, public expression
{
public:
  cast_expression(const std::vector<std::string> &tp, const riddle::ast::expression *const e);
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

class range_expression : public riddle::ast::range_expression, public expression
{
public:
  range_expression(const riddle::ast::expression *const min_e, const riddle::ast::expression *const max_e);
  range_expression(const range_expression &orig) = delete;
  virtual ~range_expression();

  expr evaluate(const scope &scp, context &ctx) const override;
};

class constructor_expression : public riddle::ast::constructor_expression, public expression
{
public:
  constructor_expression(const std::vector<std::string> &it, const std::vector<const riddle::ast::expression *> &es);
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
  lt_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r);
  lt_expression(const lt_expression &orig) = delete;
  virtual ~lt_expression();

  expr evaluate(const scope &scp, context &ctx) const override;
};

class leq_expression : public riddle::ast::leq_expression, public expression
{
public:
  leq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r);
  leq_expression(const leq_expression &orig) = delete;
  virtual ~leq_expression();

  expr evaluate(const scope &scp, context &ctx) const override;
};

class geq_expression : public riddle::ast::geq_expression, public expression
{
public:
  geq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r);
  geq_expression(const geq_expression &orig) = delete;
  virtual ~geq_expression();

  expr evaluate(const scope &scp, context &ctx) const override;
};

class gt_expression : public riddle::ast::gt_expression, public expression
{
public:
  gt_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r);
  gt_expression(const gt_expression &orig) = delete;
  virtual ~gt_expression();

  expr evaluate(const scope &scp, context &ctx) const override;
};

class function_expression : public riddle::ast::function_expression, public expression
{
public:
  function_expression(const std::vector<std::string> &is, const std::string &fn, const std::vector<const riddle::ast::expression *> &es);
  function_expression(const function_expression &orig) = delete;
  virtual ~function_expression();

  expr evaluate(const scope &scp, context &ctx) const override;
};

class id_expression : public riddle::ast::id_expression, public expression
{
public:
  id_expression(const std::vector<std::string> &is);
  id_expression(const id_expression &orig) = delete;
  virtual ~id_expression();

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
  local_field_statement(const std::vector<std::string> &ft, const std::string &n, const riddle::ast::expression *const e = nullptr);
  local_field_statement(const local_field_statement &orig) = delete;
  virtual ~local_field_statement();

  void execute(const scope &scp, context &ctx) const override;
};

class assignment_statement : public riddle::ast::assignment_statement, public statement
{
public:
  assignment_statement(const std::vector<std::string> &is, const std::string &i, const riddle::ast::expression *const e);
  assignment_statement(const assignment_statement &orig) = delete;
  virtual ~assignment_statement();

  void execute(const scope &scp, context &ctx) const override;
};

class expression_statement : public riddle::ast::expression_statement, public statement
{
public:
  expression_statement(const riddle::ast::expression *const e);
  expression_statement(const expression_statement &orig) = delete;
  virtual ~expression_statement();

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
  formula_statement(const bool &isf, const std::string &fn, const std::vector<std::string> &scp, const std::string &pn, const std::vector<std::pair<const std::string, const riddle::ast::expression *const>> &assns);
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
  riddle::ast::method_declaration *new_method_declaration(const std::vector<std::string> &rt, const std::string &n, const std::vector<std::pair<const std::vector<std::string>, const std::string>> &pars, const std::vector<const riddle::ast::statement *> &stmnts) override { return new riddle::ast::method_declaration(rt, n, pars, stmnts); }
  riddle::ast::predicate_declaration *new_predicate_declaration(const std::string &n, const std::vector<std::pair<const std::vector<std::string>, const std::string>> &pars, const std::vector<std::vector<std::string>> &pl, const std::vector<const riddle::ast::statement *> &stmnts) override { return new riddle::ast::predicate_declaration(n, pars, pl, stmnts); }
  riddle::ast::typedef_declaration *new_typedef_declaration(const std::string &n, const std::string &pt, const riddle::ast::expression *const e) override { return new riddle::ast::typedef_declaration(n, pt, e); }
  riddle::ast::enum_declaration *new_enum_declaration(const std::string &n, const std::vector<std::string> &es, const std::vector<std::vector<std::string>> &trs) override { return new riddle::ast::enum_declaration(n, es, trs); }
  riddle::ast::class_declaration *new_class_declaration(const std::string &n, const std::vector<std::vector<std::string>> &bcs, const std::vector<const riddle::ast::field_declaration *> &fs, const std::vector<const riddle::ast::constructor_declaration *> &cs, const std::vector<const riddle::ast::method_declaration *> &ms, const std::vector<const riddle::ast::predicate_declaration *> &ps, const std::vector<const riddle::ast::type_declaration *> &ts) override { return new riddle::ast::class_declaration(n, bcs, fs, cs, ms, ps, ts); }
  riddle::ast::variable_declaration *new_variable_declaration(const std::string &n, const riddle::ast::expression *const e = nullptr) override { return new riddle::ast::variable_declaration(n, e); }
  riddle::ast::field_declaration *new_field_declaration(const std::vector<std::string> &tp, const std::vector<const riddle::ast::variable_declaration *> &ds) override { return new riddle::ast::field_declaration(tp, ds); }
  riddle::ast::constructor_declaration *new_constructor_declaration(const std::vector<std::pair<const std::vector<std::string>, const std::string>> &pars, const std::vector<std::pair<const std::string, const std::vector<const riddle::ast::expression *>>> &il, const std::vector<const riddle::ast::statement *> &stmnts) override { return new riddle::ast::constructor_declaration(pars, il, stmnts); }
  riddle::ast::compilation_unit *new_compilation_unit(const std::vector<const riddle::ast::method_declaration *> &ms, const std::vector<const riddle::ast::predicate_declaration *> &ps, const std::vector<const riddle::ast::type_declaration *> &ts, const std::vector<const riddle::ast::statement *> &stmnts) override { return new riddle::ast::compilation_unit(ms, ps, ts, stmnts); }

  /**
   * The statements.
   */
  ast::local_field_statement *new_local_field_statement(const std::vector<std::string> &ft, const std::string &n, const riddle::ast::expression *const e = nullptr) override { return new ast::local_field_statement(ft, n, e); }
  ast::assignment_statement *new_assignment_statement(const std::vector<std::string> &is, const std::string &i, const riddle::ast::expression *const e) override { return new ast::assignment_statement(is, i, e); }
  ast::expression_statement *new_expression_statement(const riddle::ast::expression *const e) override { return new ast::expression_statement(e); }
  ast::disjunction_statement *new_disjunction_statement(const std::vector<std::pair<const std::vector<const riddle::ast::statement *>, const riddle::ast::expression *const>> &conjs) override { return new ast::disjunction_statement(conjs); }
  ast::conjunction_statement *new_conjunction_statement(const std::vector<const riddle::ast::statement *> &stmnts) override { return new ast::conjunction_statement(stmnts); }
  ast::formula_statement *new_formula_statement(const bool &isf, const std::string &fn, const std::vector<std::string> &scp, const std::string &pn, const std::vector<std::pair<const std::string, const riddle::ast::expression *const>> &assns) override { return new ast::formula_statement(isf, fn, scp, pn, assns); }
  ast::return_statement *new_return_statement(const riddle::ast::expression *const e) override { return new ast::return_statement(e); }

  /**
   * The expressions.
   */
  ast::bool_literal_expression *new_bool_literal_expression(const bool &l) override { return new ast::bool_literal_expression(l); }
  ast::int_literal_expression *new_int_literal_expression(const smt::I &l) override { return new ast::int_literal_expression(l); }
  ast::real_literal_expression *new_real_literal_expression(const smt::rational &l) override { return new ast::real_literal_expression(l); }
  ast::string_literal_expression *new_string_literal_expression(const std::string &l) override { return new ast::string_literal_expression(l); }
  ast::cast_expression *new_cast_expression(const std::vector<std::string> &tp, const riddle::ast::expression *const e) override { return new ast::cast_expression(tp, e); }
  ast::plus_expression *new_plus_expression(const riddle::ast::expression *const e) override { return new ast::plus_expression(e); }
  ast::minus_expression *new_minus_expression(const riddle::ast::expression *const e) override { return new ast::minus_expression(e); }
  ast::not_expression *new_not_expression(const riddle::ast::expression *const e) override { return new ast::not_expression(e); }
  ast::range_expression *new_range_expression(const riddle::ast::expression *const min_e, const riddle::ast::expression *const max_e) override { return new ast::range_expression(min_e, max_e); }
  ast::constructor_expression *new_constructor_expression(const std::vector<std::string> &it, const std::vector<const riddle::ast::expression *> &es) override { return new ast::constructor_expression(it, es); }
  ast::eq_expression *new_eq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) override { return new ast::eq_expression(l, r); }
  ast::neq_expression *new_neq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) override { return new ast::neq_expression(l, r); }
  ast::lt_expression *new_lt_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) override { return new ast::lt_expression(l, r); }
  ast::leq_expression *new_leq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) override { return new ast::leq_expression(l, r); }
  ast::geq_expression *new_geq_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) override { return new ast::geq_expression(l, r); }
  ast::gt_expression *new_gt_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) override { return new ast::gt_expression(l, r); }
  ast::function_expression *new_function_expression(const std::vector<std::string> &is, const std::string &fn, const std::vector<const riddle::ast::expression *> &es) override { return new ast::function_expression(is, fn, es); }
  ast::id_expression *new_id_expression(const std::vector<std::string> &is) override { return new ast::id_expression(is); }
  ast::implication_expression *new_implication_expression(const riddle::ast::expression *const l, const riddle::ast::expression *const r) override { return new ast::implication_expression(l, r); }
  ast::disjunction_expression *new_disjunction_expression(const std::vector<const riddle::ast::expression *> &es) override { return new ast::disjunction_expression(es); }
  ast::conjunction_expression *new_conjunction_expression(const std::vector<const riddle::ast::expression *> &es) override { return new ast::conjunction_expression(es); }
  ast::exct_one_expression *new_exct_one_expression(const std::vector<const riddle::ast::expression *> &es) override { return new ast::exct_one_expression(es); }
  ast::addition_expression *new_addition_expression(const std::vector<const riddle::ast::expression *> &es) override { return new ast::addition_expression(es); }
  ast::subtraction_expression *new_subtraction_expression(const std::vector<const riddle::ast::expression *> &es) override { return new ast::subtraction_expression(es); }
  ast::multiplication_expression *new_multiplication_expression(const std::vector<const riddle::ast::expression *> &es) override { return new ast::multiplication_expression(es); }
  ast::division_expression *new_division_expression(const std::vector<const riddle::ast::expression *> &es) override { return new ast::division_expression(es); }
};
} // namespace ratio
