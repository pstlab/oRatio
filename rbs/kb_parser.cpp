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
    } // namespace ast

    kb_parser::kb_parser(std::istream &is) : rbs::parser(is) {}
} // namespace kb
