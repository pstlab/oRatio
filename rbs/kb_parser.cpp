#include "kb_parser.h"
#include "fact.h"
#include "knowledge_base.h"
#include <algorithm>

namespace kb
{
    namespace ast
    {
        RBS_EXPORT bool_literal_expression::bool_literal_expression(const rbs::bool_token &l) : rbs::ast::bool_literal_expression(l) {}
        expr bool_literal_expression::evaluate(knowledge_base &, std::unordered_map<std::string, expr> &) const { return std::make_shared<bool_item>(literal.val); }

        RBS_EXPORT int_literal_expression::int_literal_expression(const rbs::int_token &l) : rbs::ast::int_literal_expression(l) {}
        expr int_literal_expression::evaluate(knowledge_base &, std::unordered_map<std::string, expr> &) const { return std::make_shared<arith_item>(smt::rational(literal.val)); }

        RBS_EXPORT real_literal_expression::real_literal_expression(const rbs::real_token &l) : rbs::ast::real_literal_expression(l) {}
        expr real_literal_expression::evaluate(knowledge_base &, std::unordered_map<std::string, expr> &) const { return std::make_shared<arith_item>(literal.val); }

        RBS_EXPORT string_literal_expression::string_literal_expression(const rbs::string_token &l) : rbs::ast::string_literal_expression(l) {}
        expr string_literal_expression::evaluate(knowledge_base &, std::unordered_map<std::string, expr> &) const { return std::make_shared<string_item>(literal.str); }

        plus_expression::plus_expression(const rbs::ast::expression *const e) : rbs::ast::plus_expression(e) {}
        expr plus_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const { return static_cast<const ast::expression *>(xpr)->evaluate(kb, ctx); }

        minus_expression::minus_expression(const rbs::ast::expression *const e) : rbs::ast::minus_expression(e) {}
        expr minus_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const { return std::make_shared<arith_item>(-static_cast<arith_item *>(static_cast<const ast::expression *>(xpr)->evaluate(kb, ctx).get())->get()); }

        not_expression::not_expression(const rbs::ast::expression *const e) : rbs::ast::not_expression(e) {}
        expr not_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const { return std::make_shared<bool_item>(!static_cast<bool_item *>(static_cast<const ast::expression *>(xpr)->evaluate(kb, ctx).get())->get()); }

        eq_expression::eq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::eq_expression(l, r) {}
        expr eq_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const { return {std::make_shared<bool_item>(static_cast<const ast::expression *>(left)->evaluate(kb, ctx)->eq(*static_cast<const ast::expression *>(right)->evaluate(kb, ctx)))}; }

        neq_expression::neq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::neq_expression(l, r) {}
        expr neq_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const { return std::make_shared<bool_item>(!static_cast<const ast::expression *>(left)->evaluate(kb, ctx)->eq(*static_cast<const ast::expression *>(right)->evaluate(kb, ctx))); }

        RBS_EXPORT lt_expression::lt_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::lt_expression(l, r) {}
        expr lt_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const { return std::make_shared<bool_item>(static_cast<arith_item *>(static_cast<const ast::expression *>(left)->evaluate(kb, ctx).get()) < static_cast<arith_item *>(static_cast<const ast::expression *>(right)->evaluate(kb, ctx).get())); }

        RBS_EXPORT leq_expression::leq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::leq_expression(l, r) {}
        expr leq_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const { return std::make_shared<bool_item>(static_cast<arith_item *>(static_cast<const ast::expression *>(left)->evaluate(kb, ctx).get()) <= static_cast<arith_item *>(static_cast<const ast::expression *>(right)->evaluate(kb, ctx).get())); }

        RBS_EXPORT geq_expression::geq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::geq_expression(l, r) {}
        expr geq_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const { return std::make_shared<bool_item>(static_cast<arith_item *>(static_cast<const ast::expression *>(left)->evaluate(kb, ctx).get()) >= static_cast<arith_item *>(static_cast<const ast::expression *>(right)->evaluate(kb, ctx).get())); }

        RBS_EXPORT gt_expression::gt_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::gt_expression(l, r) {}
        expr gt_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const { return std::make_shared<bool_item>(static_cast<arith_item *>(static_cast<const ast::expression *>(left)->evaluate(kb, ctx).get()) > static_cast<arith_item *>(static_cast<const ast::expression *>(right)->evaluate(kb, ctx).get())); }

        RBS_EXPORT id_expression::id_expression(const std::vector<rbs::id_token> &is) : rbs::ast::id_expression(is) {}
        expr id_expression::evaluate(knowledge_base &, std::unordered_map<std::string, expr> &ctx) const { return ids.size() == 1 ? ctx.at(ids.at(0).id) : static_cast<fact *>(ctx.at(ids.at(0).id).get())->get(ids.at(1).id); }

        implication_expression::implication_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::implication_expression(l, r) {}
        expr implication_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const { return std::make_shared<bool_item>(!static_cast<bool_item *>(static_cast<const ast::expression *>(left)->evaluate(kb, ctx).get()) || static_cast<bool_item *>(static_cast<const ast::expression *>(right)->evaluate(kb, ctx).get())); }

        disjunction_expression::disjunction_expression(const std::vector<const rbs::ast::expression *> &es) : rbs::ast::disjunction_expression(es) {}
        expr disjunction_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const
        {
            return std::make_shared<bool_item>(std::any_of(expressions.cbegin(), expressions.cend(), [&kb, &ctx](const auto &xpr)
                                                           { return static_cast<bool_item *>(static_cast<const ast::expression *>(xpr)->evaluate(kb, ctx).get())->get(); }));
        }

        conjunction_expression::conjunction_expression(const std::vector<const rbs::ast::expression *> &es) : rbs::ast::conjunction_expression(es) {}
        expr conjunction_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const
        {
            return std::make_shared<bool_item>(std::all_of(expressions.cbegin(), expressions.cend(), [&kb, &ctx](const auto &xpr)
                                                           { return static_cast<bool_item *>(static_cast<const ast::expression *>(xpr)->evaluate(kb, ctx).get())->get(); }));
        }

        exct_one_expression::exct_one_expression(const std::vector<const rbs::ast::expression *> &es) : rbs::ast::exct_one_expression(es) {}
        expr exct_one_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const
        {
            size_t trues = 0;
            for (const auto &xpr : expressions)
                if (static_cast<bool_item *>(static_cast<const ast::expression *>(xpr)->evaluate(kb, ctx).get())->get())
                    trues++;
            return std::make_shared<bool_item>(trues == 1);
        }

        addition_expression::addition_expression(const std::vector<const rbs::ast::expression *> &es) : rbs::ast::addition_expression(es) {}
        expr addition_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const
        {
            smt::rational sum;
            for (const auto &xpr : expressions)
                sum += static_cast<arith_item *>(static_cast<const ast::expression *>(xpr)->evaluate(kb, ctx).get())->get();
            return std::make_shared<arith_item>(sum);
        }

        subtraction_expression::subtraction_expression(const std::vector<const rbs::ast::expression *> &es) : rbs::ast::subtraction_expression(es) {}
        expr subtraction_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const
        {
            smt::rational sub;
            for (const auto &xpr : expressions)
                sub -= static_cast<arith_item *>(static_cast<const ast::expression *>(xpr)->evaluate(kb, ctx).get())->get();
            return std::make_shared<arith_item>(sub);
        }

        multiplication_expression::multiplication_expression(const std::vector<const rbs::ast::expression *> &es) : rbs::ast::multiplication_expression(es) {}
        expr multiplication_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const
        {
            smt::rational mult;
            for (const auto &xpr : expressions)
                mult *= static_cast<arith_item *>(static_cast<const ast::expression *>(xpr)->evaluate(kb, ctx).get())->get();
            return std::make_shared<arith_item>(mult);
        }

        division_expression::division_expression(const std::vector<const rbs::ast::expression *> &es) : rbs::ast::division_expression(es) {}
        expr division_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const
        {
            smt::rational div;
            for (const auto &xpr : expressions)
                div /= static_cast<arith_item *>(static_cast<const ast::expression *>(xpr)->evaluate(kb, ctx).get())->get();
            return std::make_shared<arith_item>(div);
        }

        fact_expression::fact_expression(const rbs::id_token &fn, const rbs::id_token &pn, const std::vector<std::pair<const rbs::id_token, const rbs::ast::expression *const>> &assns) : rbs::ast::fact_expression(fn, pn, assns) {}
        expr fact_expression::evaluate(knowledge_base &kb, std::unordered_map<std::string, expr> &ctx) const
        {
            if (kb.exists_predicate(predicate_name.id))
            {
            }
            return std::make_shared<bool_item>(false);
        }
    } // namespace ast

    kb_parser::kb_parser(std::istream &is) : rbs::parser(is) {}
} // namespace kb
