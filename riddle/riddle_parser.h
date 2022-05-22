#pragma once

#include "riddle_lexer.h"
#include <vector>

namespace riddle
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
      RIDDLE_EXPORT bool_literal_expression(const bool_token &l) : literal(l) {}
      bool_literal_expression(const bool_literal_expression &orig) = delete;
      RIDDLE_EXPORT virtual ~bool_literal_expression() = default;

    protected:
      const bool_token literal;
    };

    class int_literal_expression : public expression
    {
    public:
      RIDDLE_EXPORT int_literal_expression(const int_token &l) : literal(l) {}
      int_literal_expression(const int_literal_expression &orig) = delete;
      RIDDLE_EXPORT virtual ~int_literal_expression() = default;

    protected:
      const int_token literal;
    };

    class real_literal_expression : public expression
    {
    public:
      RIDDLE_EXPORT real_literal_expression(const real_token &l) : literal(l) {}
      real_literal_expression(const real_literal_expression &orig) = delete;
      RIDDLE_EXPORT virtual ~real_literal_expression() = default;

    protected:
      const real_token literal;
    };

    class string_literal_expression : public expression
    {
    public:
      RIDDLE_EXPORT string_literal_expression(const string_token &l) : literal(l) {}
      string_literal_expression(const string_literal_expression &orig) = delete;
      RIDDLE_EXPORT virtual ~string_literal_expression() = default;

    protected:
      const string_token literal;
    };

    class cast_expression : public expression
    {
    public:
      cast_expression(const std::vector<id_token> &tp, const expression *const e) : cast_to_type(tp), xpr(e) {}
      cast_expression(const cast_expression &orig) = delete;
      virtual ~cast_expression() { delete xpr; }

    protected:
      const std::vector<id_token> cast_to_type;
      const expression *const xpr;
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

    class constructor_expression : public expression
    {
    public:
      constructor_expression(const std::vector<id_token> &it, const std::vector<const expression *> &es) : instance_type(it), expressions(es) {}
      constructor_expression(const constructor_expression &orig) = delete;
      virtual ~constructor_expression()
      {
        for (const auto &e : expressions)
          delete e;
      }

    protected:
      const std::vector<id_token> instance_type;
      const std::vector<const expression *> expressions;
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
      RIDDLE_EXPORT lt_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
      lt_expression(const lt_expression &orig) = delete;
      RIDDLE_EXPORT virtual ~lt_expression()
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
      RIDDLE_EXPORT leq_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
      leq_expression(const leq_expression &orig) = delete;
      RIDDLE_EXPORT virtual ~leq_expression()
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
      RIDDLE_EXPORT geq_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
      geq_expression(const geq_expression &orig) = delete;
      RIDDLE_EXPORT virtual ~geq_expression()
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
      RIDDLE_EXPORT gt_expression(const expression *const l, const expression *const r) : left(l), right(r) {}
      gt_expression(const gt_expression &orig) = delete;
      RIDDLE_EXPORT virtual ~gt_expression()
      {
        delete left;
        delete right;
      }

    protected:
      const expression *const left;
      const expression *const right;
    };

    class function_expression : public expression
    {
    public:
      function_expression(const std::vector<id_token> &is, const id_token &fn, const std::vector<const expression *> &es) : ids(is), function_name(fn), expressions(es) {}
      function_expression(const function_expression &orig) = delete;
      virtual ~function_expression()
      {
        for (const auto &e : expressions)
          delete e;
      }

    protected:
      const std::vector<id_token> ids;
      const id_token function_name;
      const std::vector<const expression *> expressions;
    };

    class id_expression : public expression
    {
    public:
      RIDDLE_EXPORT id_expression(const std::vector<id_token> &is) : ids(is) {}
      id_expression(const id_expression &orig) = delete;
      RIDDLE_EXPORT virtual ~id_expression() = default;

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

    class statement
    {
    public:
      statement() = default;
      statement(const statement &orig) = delete;
      virtual ~statement() = default;
    };

    class local_field_statement : public statement
    {
    public:
      local_field_statement(const std::vector<id_token> &ft, const std::vector<id_token> &ns, const std::vector<const expression *> &es) : field_type(ft), names(ns), xprs(es) {}
      local_field_statement(const local_field_statement &orig) = delete;
      virtual ~local_field_statement() = default;

    protected:
      const std::vector<id_token> field_type;
      const std::vector<id_token> names;
      const std::vector<const expression *> xprs;
    };

    class assignment_statement : public statement
    {
    public:
      assignment_statement(const std::vector<id_token> &is, const id_token &i, const expression *const e) : ids(is), id(i), xpr(e) {}
      assignment_statement(const assignment_statement &orig) = delete;
      virtual ~assignment_statement() { delete xpr; }

    protected:
      const std::vector<id_token> ids;
      const id_token id;
      const expression *const xpr;
    };

    class expression_statement : public statement
    {
    public:
      RIDDLE_EXPORT expression_statement(const expression *const e) : xpr(e) {}
      expression_statement(const expression_statement &orig) = delete;
      RIDDLE_EXPORT virtual ~expression_statement() { delete xpr; }

    protected:
      const expression *const xpr;
    };

    class disjunction_statement : public statement
    {
    public:
      disjunction_statement(const std::vector<std::pair<const std::vector<const statement *>, const expression *const>> &conjs) : conjunctions(conjs) {}
      disjunction_statement(const disjunction_statement &orig) = delete;
      virtual ~disjunction_statement()
      {
        for (const auto &[stmnts, xpr] : conjunctions)
        {
          for (const auto &s : stmnts)
            delete s;
          delete xpr;
        }
      }

    protected:
      const std::vector<std::pair<const std::vector<const statement *>, const expression *const>> conjunctions;
    };

    class conjunction_statement : public statement
    {
    public:
      conjunction_statement(const std::vector<const statement *> &stmnts) : statements(stmnts) {}
      conjunction_statement(const conjunction_statement &orig) = delete;
      virtual ~conjunction_statement()
      {
        for (const auto &st : statements)
          delete st;
      }

    protected:
      const std::vector<const statement *> statements;
    };

    class formula_statement : public statement
    {
    public:
      formula_statement(const bool &isf, const id_token &fn, const std::vector<id_token> &scp, const id_token &pn, const std::vector<std::pair<const id_token, const expression *const>> &assns) : is_fact(isf), formula_name(fn), formula_scope(scp), predicate_name(pn), assignments(assns) {}
      formula_statement(const formula_statement &orig) = delete;
      virtual ~formula_statement()
      {
        for ([[maybe_unused]] const auto &[id_tkn, xpr] : assignments)
          delete xpr;
      }

    protected:
      const bool is_fact;
      const id_token formula_name;
      const std::vector<id_token> formula_scope;
      const id_token predicate_name;
      const std::vector<std::pair<const id_token, const expression *const>> assignments;
    };

    class return_statement : public statement
    {
    public:
      return_statement(const expression *const e) : xpr(e) {}
      return_statement(const return_statement &orig) = delete;
      virtual ~return_statement() { delete xpr; }

    protected:
      const expression *const xpr;
    };

    class type_declaration
    {
    public:
      type_declaration() {}
      type_declaration(const type_declaration &orig) = delete;
      virtual ~type_declaration() {}
    };

    class method_declaration
    {
    public:
      method_declaration(const std::vector<id_token> &rt, const id_token &n, const std::vector<std::pair<const std::vector<id_token>, const id_token>> &pars, const std::vector<const statement *> &stmnts) : return_type(rt), name(n), parameters(pars), statements(stmnts) {}
      method_declaration(const method_declaration &orig) = delete;
      virtual ~method_declaration()
      {
        for (const auto &s : statements)
          delete s;
      }

    protected:
      const std::vector<id_token> return_type;
      const id_token name;
      const std::vector<std::pair<const std::vector<id_token>, const id_token>> parameters;
      const std::vector<const statement *> statements;
    };

    class predicate_declaration
    {
    public:
      predicate_declaration(const id_token &n, const std::vector<std::pair<const std::vector<id_token>, const id_token>> &pars, const std::vector<std::vector<id_token>> &pl, const std::vector<const statement *> &stmnts) : name(n), parameters(pars), predicate_list(pl), statements(stmnts) {}
      predicate_declaration(const predicate_declaration &orig) = delete;
      virtual ~predicate_declaration()
      {
        for (const auto &s : statements)
          delete s;
      }

    protected:
      const id_token name;
      const std::vector<std::pair<const std::vector<id_token>, const id_token>> parameters;
      const std::vector<std::vector<id_token>> predicate_list;
      const std::vector<const statement *> statements;
    };

    class typedef_declaration : public type_declaration
    {
    public:
      typedef_declaration(const id_token &n, const id_token &pt, const expression *const e) : name(n), primitive_type(pt), xpr(e) {}
      typedef_declaration(const typedef_declaration &orig) = delete;
      virtual ~typedef_declaration() { delete xpr; }

    protected:
      const id_token name;
      const id_token primitive_type;
      const expression *const xpr;
    };

    class enum_declaration : public type_declaration
    {
    public:
      enum_declaration(const id_token &n, const std::vector<string_token> &es, const std::vector<std::vector<id_token>> &trs) : name(n), enums(es), type_refs(trs) {}
      enum_declaration(const enum_declaration &orig) = delete;
      virtual ~enum_declaration() = default;

    protected:
      const id_token name;
      const std::vector<string_token> enums;
      const std::vector<std::vector<id_token>> type_refs;
    };

    class variable_declaration
    {
      friend class field_declaration;

    public:
      variable_declaration(const id_token &n, const expression *const e = nullptr) : name(n), xpr(e) {}
      variable_declaration(const variable_declaration &orig) = delete;
      virtual ~variable_declaration() { delete xpr; }

    protected:
      const id_token name;
      const expression *const xpr;
    };

    class field_declaration
    {
    public:
      field_declaration(const std::vector<id_token> &tp, const std::vector<const variable_declaration *> &ds) : field_type(tp), declarations(ds) {}
      field_declaration(const field_declaration &orig) = delete;
      virtual ~field_declaration()
      {
        for (const auto &vd : declarations)
          delete vd;
      }

    protected:
      const std::vector<id_token> field_type;
      const std::vector<const variable_declaration *> declarations;
    };

    class constructor_declaration
    {
    public:
      constructor_declaration(const std::vector<std::pair<const std::vector<id_token>, const id_token>> &pars, const std::vector<std::pair<const id_token, const std::vector<const expression *>>> &il, const std::vector<const statement *> &stmnts) : parameters(pars), init_list(il), statements(stmnts) {}
      constructor_declaration(const constructor_declaration &orig) = delete;
      virtual ~constructor_declaration()
      {
        for ([[maybe_unused]] const auto &[id_tkn, xprs] : init_list)
          for (const auto &xpr : xprs)
            delete xpr;
        for (const auto &s : statements)
          delete s;
      }

    protected:
      const std::vector<std::pair<const std::vector<id_token>, const id_token>> parameters;
      const std::vector<std::pair<const id_token, const std::vector<const expression *>>> init_list;
      const std::vector<const statement *> statements;
    };

    class class_declaration : public type_declaration
    {
    public:
      class_declaration(const id_token &n, const std::vector<std::vector<id_token>> &bcs, const std::vector<const field_declaration *> &fs, const std::vector<const constructor_declaration *> &cs, const std::vector<const method_declaration *> &ms, const std::vector<const predicate_declaration *> &ps, const std::vector<const type_declaration *> &ts) : name(n), base_classes(bcs), fields(fs), constructors(cs), methods(ms), predicates(ps), types(ts) {}
      class_declaration(const class_declaration &orig) = delete;
      virtual ~class_declaration()
      {
        for (const auto &f : fields)
          delete f;
        for (const auto &c : constructors)
          delete c;
        for (const auto &m : methods)
          delete m;
        for (const auto &p : predicates)
          delete p;
        for (const auto &t : types)
          delete t;
      }

    protected:
      const id_token name;
      const std::vector<std::vector<id_token>> base_classes;
      const std::vector<const field_declaration *> fields;
      const std::vector<const constructor_declaration *> constructors;
      const std::vector<const method_declaration *> methods;
      const std::vector<const predicate_declaration *> predicates;
      const std::vector<const type_declaration *> types;
    };

    class compilation_unit
    {
    public:
      compilation_unit(const std::vector<const method_declaration *> &ms, const std::vector<const predicate_declaration *> &ps, const std::vector<const type_declaration *> &ts, const std::vector<const statement *> &stmnts) : methods(ms), predicates(ps), types(ts), statements(stmnts) {}
      compilation_unit(const compilation_unit &orig) = delete;
      virtual ~compilation_unit()
      {
        for (const auto &t : types)
          delete t;
        for (const auto &m : methods)
          delete m;
        for (const auto &p : predicates)
          delete p;
        for (const auto &s : statements)
          delete s;
      }

    protected:
      const std::vector<const method_declaration *> methods;
      const std::vector<const predicate_declaration *> predicates;
      const std::vector<const type_declaration *> types;
      const std::vector<const statement *> statements;
    };
  } // namespace ast

  class parser
  {
  public:
    RIDDLE_EXPORT parser(std::istream &is);
    parser(const parser &orig) = delete;
    RIDDLE_EXPORT virtual ~parser();

    RIDDLE_EXPORT ast::compilation_unit *parse();

  private:
    token *next();
    bool match(const symbol &sym);
    void backtrack(const size_t &p) noexcept;

    ast::typedef_declaration *_typedef_declaration();
    ast::enum_declaration *_enum_declaration();
    ast::class_declaration *_class_declaration();
    ast::field_declaration *_field_declaration();
    ast::method_declaration *_method_declaration();
    ast::constructor_declaration *_constructor_declaration();
    ast::predicate_declaration *_predicate_declaration();
    ast::statement *_statement();
    ast::expression *_expression(const size_t &pr = 0);

    void error(const std::string &err);

    /**
     * The declarations.
     */
    virtual ast::method_declaration *new_method_declaration(const std::vector<id_token> &rt, const id_token &n, const std::vector<std::pair<const std::vector<id_token>, const id_token>> &pars, const std::vector<const ast::statement *> &stmnts) const noexcept { return new ast::method_declaration(rt, n, pars, stmnts); }
    virtual ast::predicate_declaration *new_predicate_declaration(const id_token &n, const std::vector<std::pair<const std::vector<id_token>, const id_token>> &pars, const std::vector<std::vector<id_token>> &pl, const std::vector<const ast::statement *> &stmnts) const noexcept { return new ast::predicate_declaration(n, pars, pl, stmnts); }
    virtual ast::typedef_declaration *new_typedef_declaration(const id_token &n, const id_token &pt, const ast::expression *e) const noexcept { return new ast::typedef_declaration(n, pt, e); }
    virtual ast::enum_declaration *new_enum_declaration(const id_token &n, const std::vector<string_token> &es, const std::vector<std::vector<id_token>> &trs) const noexcept { return new ast::enum_declaration(n, es, trs); }
    virtual ast::class_declaration *new_class_declaration(const id_token &n, const std::vector<std::vector<id_token>> &bcs, const std::vector<const ast::field_declaration *> &fs, const std::vector<const ast::constructor_declaration *> &cs, const std::vector<const ast::method_declaration *> &ms, const std::vector<const ast::predicate_declaration *> &ps, const std::vector<const ast::type_declaration *> &ts) const noexcept { return new ast::class_declaration(n, bcs, fs, cs, ms, ps, ts); }
    virtual ast::variable_declaration *new_variable_declaration(const id_token &n, const ast::expression *const e = nullptr) const noexcept { return new ast::variable_declaration(n, e); }
    virtual ast::field_declaration *new_field_declaration(const std::vector<id_token> &tp, const std::vector<const ast::variable_declaration *> &ds) const noexcept { return new ast::field_declaration(tp, ds); }
    virtual ast::constructor_declaration *new_constructor_declaration(const std::vector<std::pair<const std::vector<id_token>, const id_token>> &pars, const std::vector<std::pair<const id_token, const std::vector<const ast::expression *>>> &il, const std::vector<const ast::statement *> &stmnts) const noexcept { return new ast::constructor_declaration(pars, il, stmnts); }
    virtual ast::compilation_unit *new_compilation_unit(const std::vector<const ast::method_declaration *> &ms, const std::vector<const ast::predicate_declaration *> &ps, const std::vector<const ast::type_declaration *> &ts, const std::vector<const ast::statement *> &stmnts) const noexcept { return new ast::compilation_unit(ms, ps, ts, stmnts); }

    /**
     * The statements.
     */
    virtual ast::local_field_statement *new_local_field_statement(const std::vector<id_token> &ft, const std::vector<id_token> &ns, const std::vector<const ast::expression *> &es) const noexcept { return new ast::local_field_statement(ft, ns, es); }
    virtual ast::assignment_statement *new_assignment_statement(const std::vector<id_token> &is, const id_token &i, const ast::expression *const e) const noexcept { return new ast::assignment_statement(is, i, e); }
    virtual ast::expression_statement *new_expression_statement(const ast::expression *const e) const noexcept { return new ast::expression_statement(e); }
    virtual ast::disjunction_statement *new_disjunction_statement(const std::vector<std::pair<const std::vector<const ast::statement *>, const ast::expression *const>> &conjs) const noexcept { return new ast::disjunction_statement(conjs); }
    virtual ast::conjunction_statement *new_conjunction_statement(const std::vector<const ast::statement *> &stmnts) const noexcept { return new ast::conjunction_statement(stmnts); }
    virtual ast::formula_statement *new_formula_statement(const bool &isf, const id_token &fn, const std::vector<id_token> &scp, const id_token &pn, const std::vector<std::pair<const id_token, const ast::expression *const>> &assns) const noexcept { return new ast::formula_statement(isf, fn, scp, pn, assns); }
    virtual ast::return_statement *new_return_statement(const ast::expression *const e) const noexcept { return new ast::return_statement(e); }

    /**
     * The expressions.
     */
    virtual ast::bool_literal_expression *new_bool_literal_expression(const bool_token &l) const noexcept { return new ast::bool_literal_expression(l); }
    virtual ast::int_literal_expression *new_int_literal_expression(const int_token &l) const noexcept { return new ast::int_literal_expression(l); }
    virtual ast::real_literal_expression *new_real_literal_expression(const real_token &l) const noexcept { return new ast::real_literal_expression(l); }
    virtual ast::string_literal_expression *new_string_literal_expression(const string_token &l) const noexcept { return new ast::string_literal_expression(l); }
    virtual ast::cast_expression *new_cast_expression(const std::vector<id_token> &tp, const ast::expression *const e) const noexcept { return new ast::cast_expression(tp, e); }
    virtual ast::plus_expression *new_plus_expression(const ast::expression *const e) const noexcept { return new ast::plus_expression(e); }
    virtual ast::minus_expression *new_minus_expression(const ast::expression *const e) const noexcept { return new ast::minus_expression(e); }
    virtual ast::not_expression *new_not_expression(const ast::expression *const e) const noexcept { return new ast::not_expression(e); }
    virtual ast::constructor_expression *new_constructor_expression(const std::vector<id_token> &it, const std::vector<const ast::expression *> &es) const noexcept { return new ast::constructor_expression(it, es); }
    virtual ast::eq_expression *new_eq_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::eq_expression(l, r); }
    virtual ast::neq_expression *new_neq_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::neq_expression(l, r); }
    virtual ast::lt_expression *new_lt_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::lt_expression(l, r); }
    virtual ast::leq_expression *new_leq_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::leq_expression(l, r); }
    virtual ast::geq_expression *new_geq_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::geq_expression(l, r); }
    virtual ast::gt_expression *new_gt_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::gt_expression(l, r); }
    virtual ast::function_expression *new_function_expression(const std::vector<id_token> &is, const id_token &fn, const std::vector<const ast::expression *> &es) const noexcept { return new ast::function_expression(is, fn, es); }
    virtual ast::id_expression *new_id_expression(const std::vector<id_token> &is) const noexcept { return new ast::id_expression(is); }
    virtual ast::implication_expression *new_implication_expression(const ast::expression *const l, const ast::expression *const r) const noexcept { return new ast::implication_expression(l, r); }
    virtual ast::disjunction_expression *new_disjunction_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::disjunction_expression(es); }
    virtual ast::conjunction_expression *new_conjunction_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::conjunction_expression(es); }
    virtual ast::exct_one_expression *new_exct_one_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::exct_one_expression(es); }
    virtual ast::addition_expression *new_addition_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::addition_expression(es); }
    virtual ast::subtraction_expression *new_subtraction_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::subtraction_expression(es); }
    virtual ast::multiplication_expression *new_multiplication_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::multiplication_expression(es); }
    virtual ast::division_expression *new_division_expression(const std::vector<const ast::expression *> &es) const noexcept { return new ast::division_expression(es); }

  private:
    lexer lex;                // the current lexer..
    token *tk = nullptr;      // the current lookahead token..
    std::vector<token *> tks; // all the tokens parsed so far..
    size_t pos = 0;           // the current position within tks'..
  };
} // namespace riddle