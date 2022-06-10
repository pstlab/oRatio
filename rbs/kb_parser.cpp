#include "kb_parser.h"

namespace kb
{
    namespace ast
    {
        RBS_EXPORT bool_literal_expression::bool_literal_expression(const rbs::bool_token &l) : rbs::ast::bool_literal_expression(l) {}
        expr bool_literal_expression::evaluate(knowledge_base &) const { return std::make_shared<bool_item>(literal.val); }

        RBS_EXPORT int_literal_expression::int_literal_expression(const rbs::int_token &l) : rbs::ast::int_literal_expression(l) {}
        expr int_literal_expression::evaluate(knowledge_base &) const { return std::make_shared<arith_item>(smt::rational(literal.val)); }

        RBS_EXPORT real_literal_expression::real_literal_expression(const rbs::real_token &l) : rbs::ast::real_literal_expression(l) {}
        expr real_literal_expression::evaluate(knowledge_base &) const { return std::make_shared<arith_item>(literal.val); }

        RBS_EXPORT string_literal_expression::string_literal_expression(const rbs::string_token &l) : rbs::ast::string_literal_expression(l) {}
        expr string_literal_expression::evaluate(knowledge_base &) const { return std::make_shared<string_item>(literal.str); }

        plus_expression::plus_expression(const rbs::ast::expression *const e) : rbs::ast::plus_expression(e) {}
        expr plus_expression::evaluate(knowledge_base &kb) const { return dynamic_cast<const ast::expression *>(xpr)->evaluate(kb); }

        minus_expression::minus_expression(const rbs::ast::expression *const e) : rbs::ast::minus_expression(e) {}
        expr minus_expression::evaluate(knowledge_base &kb) const { return std::make_shared<arith_item>(-static_cast<arith_item *>(dynamic_cast<const ast::expression *>(xpr)->evaluate(kb).get())->get()); }

        not_expression::not_expression(const rbs::ast::expression *const e) : rbs::ast::not_expression(e) {}
        expr not_expression::evaluate(knowledge_base &kb) const { return std::make_shared<bool_item>(!static_cast<bool_item *>(dynamic_cast<const ast::expression *>(xpr)->evaluate(kb).get())->get()); }

        eq_expression::eq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::eq_expression(l, r) {}
        expr eq_expression::evaluate(knowledge_base &kb) const
        {
            auto l = dynamic_cast<const ast::expression *const>(left)->evaluate(kb);
            auto r = dynamic_cast<const ast::expression *const>(right)->evaluate(kb);
            return std::make_shared<bool_item>(l->eq(*r));
        }

        neq_expression::neq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::neq_expression(l, r) {}
        expr neq_expression::evaluate(knowledge_base &kb) const
        {
            auto l = dynamic_cast<const ast::expression *const>(left)->evaluate(kb);
            auto r = dynamic_cast<const ast::expression *const>(right)->evaluate(kb);
            return std::make_shared<bool_item>(!l->eq(*r));
        }

        RBS_EXPORT lt_expression::lt_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::lt_expression(l, r) {}
        expr lt_expression::evaluate(knowledge_base &kb) const
        {
            auto l = dynamic_cast<const ast::expression *const>(left)->evaluate(kb);
            auto r = dynamic_cast<const ast::expression *const>(right)->evaluate(kb);
            return std::make_shared<bool_item>(static_cast<arith_item *>(l.get()) < static_cast<arith_item *>(r.get()));
        }

        RBS_EXPORT leq_expression::leq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::leq_expression(l, r) {}
        expr leq_expression::evaluate(knowledge_base &kb) const
        {
            auto l = dynamic_cast<const ast::expression *const>(left)->evaluate(kb);
            auto r = dynamic_cast<const ast::expression *const>(right)->evaluate(kb);
            return std::make_shared<bool_item>(static_cast<arith_item *>(l.get()) <= static_cast<arith_item *>(r.get()));
        }

        RBS_EXPORT geq_expression::geq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::geq_expression(l, r) {}
        expr geq_expression::evaluate(knowledge_base &kb) const
        {
            auto l = dynamic_cast<const ast::expression *const>(left)->evaluate(kb);
            auto r = dynamic_cast<const ast::expression *const>(right)->evaluate(kb);
            return std::make_shared<bool_item>(static_cast<arith_item *>(l.get()) >= static_cast<arith_item *>(r.get()));
        }

        RBS_EXPORT gt_expression::gt_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::gt_expression(l, r) {}
        expr gt_expression::evaluate(knowledge_base &kb) const
        {
            auto l = dynamic_cast<const ast::expression *const>(left)->evaluate(kb);
            auto r = dynamic_cast<const ast::expression *const>(right)->evaluate(kb);
            return std::make_shared<bool_item>(static_cast<arith_item *>(l.get()) > static_cast<arith_item *>(r.get()));
        }
    } // namespace ast

    kb_parser::kb_parser(std::istream &is) : rbs::parser(is) {}
} // namespace kb
