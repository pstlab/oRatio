#include "kb_parser.h"
#include "fact.h"

namespace kb
{
    namespace ast
    {
        RBS_EXPORT bool_literal_expression::bool_literal_expression(const rbs::bool_token &l) : rbs::ast::bool_literal_expression(l) {}
        std::vector<std::vector<expr>> bool_literal_expression::evaluate(std::unordered_map<std::string, expr> &) const { return {{std::make_shared<bool_item>(literal.val)}}; }

        RBS_EXPORT int_literal_expression::int_literal_expression(const rbs::int_token &l) : rbs::ast::int_literal_expression(l) {}
        std::vector<std::vector<expr>> int_literal_expression::evaluate(std::unordered_map<std::string, expr> &) const { return {{std::make_shared<arith_item>(smt::rational(literal.val))}}; }

        RBS_EXPORT real_literal_expression::real_literal_expression(const rbs::real_token &l) : rbs::ast::real_literal_expression(l) {}
        std::vector<std::vector<expr>> real_literal_expression::evaluate(std::unordered_map<std::string, expr> &) const { return {{std::make_shared<arith_item>(literal.val)}}; }

        RBS_EXPORT string_literal_expression::string_literal_expression(const rbs::string_token &l) : rbs::ast::string_literal_expression(l) {}
        std::vector<std::vector<expr>> string_literal_expression::evaluate(std::unordered_map<std::string, expr> &) const { return {{std::make_shared<string_item>(literal.str)}}; }

        plus_expression::plus_expression(const rbs::ast::expression *const e) : rbs::ast::plus_expression(e) {}
        std::vector<std::vector<expr>> plus_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const { return dynamic_cast<const ast::expression *>(xpr)->evaluate(ctx); }

        minus_expression::minus_expression(const rbs::ast::expression *const e) : rbs::ast::minus_expression(e) {}
        std::vector<std::vector<expr>> minus_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const { return {{std::make_shared<arith_item>(-static_cast<arith_item *>(dynamic_cast<const ast::expression *>(xpr)->evaluate(ctx).at(0).at(0).get())->get())}}; }

        not_expression::not_expression(const rbs::ast::expression *const e) : rbs::ast::not_expression(e) {}
        std::vector<std::vector<expr>> not_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const
        {
            std::vector<std::vector<expr>> c_xprs = dynamic_cast<const ast::expression *>(xpr)->evaluate(ctx);
            std::vector<std::vector<expr>> xprs;
            xprs.reserve(c_xprs.size());
            for (const auto &xpr : c_xprs)
                xprs.push_back();

            return std::make_shared<bool_item>(!static_cast<bool_item *>(dynamic_cast<const ast::expression *>(xpr)->evaluate(ctx).get())->get());
        }

        eq_expression::eq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::eq_expression(l, r) {}
        std::vector<std::vector<expr>> eq_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const
        {
            auto l = dynamic_cast<const ast::expression *const>(left)->evaluate(ctx);
            auto r = dynamic_cast<const ast::expression *const>(right)->evaluate(ctx);
            return {std::make_shared<bool_item>(l.at(0)->eq(*r.at(0)))};
        }

        neq_expression::neq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::neq_expression(l, r) {}
        std::vector<std::vector<expr>> neq_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const
        {
            auto l = dynamic_cast<const ast::expression *const>(left)->evaluate(ctx);
            auto r = dynamic_cast<const ast::expression *const>(right)->evaluate(ctx);
            return std::make_shared<bool_item>(!l.at(0)->eq(*r.at(0)));
        }

        RBS_EXPORT lt_expression::lt_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::lt_expression(l, r) {}
        std::vector<std::vector<expr>> lt_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const
        {
            auto l = dynamic_cast<const ast::expression *const>(left)->evaluate(ctx);
            auto r = dynamic_cast<const ast::expression *const>(right)->evaluate(ctx);
            return std::make_shared<bool_item>(static_cast<arith_item *>(l.get()) < static_cast<arith_item *>(r.get()));
        }

        RBS_EXPORT leq_expression::leq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::leq_expression(l, r) {}
        std::vector<std::vector<expr>> leq_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const
        {
            auto l = dynamic_cast<const ast::expression *const>(left)->evaluate(ctx);
            auto r = dynamic_cast<const ast::expression *const>(right)->evaluate(ctx);
            return std::make_shared<bool_item>(static_cast<arith_item *>(l.get()) <= static_cast<arith_item *>(r.get()));
        }

        RBS_EXPORT geq_expression::geq_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::geq_expression(l, r) {}
        std::vector<std::vector<expr>> geq_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const
        {
            auto l = dynamic_cast<const ast::expression *const>(left)->evaluate(ctx);
            auto r = dynamic_cast<const ast::expression *const>(right)->evaluate(ctx);
            return std::make_shared<bool_item>(static_cast<arith_item *>(l.get()) >= static_cast<arith_item *>(r.get()));
        }

        RBS_EXPORT gt_expression::gt_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::gt_expression(l, r) {}
        std::vector<std::vector<expr>> gt_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const
        {
            auto l = dynamic_cast<const ast::expression *const>(left)->evaluate(ctx);
            auto r = dynamic_cast<const ast::expression *const>(right)->evaluate(ctx);
            return std::make_shared<bool_item>(static_cast<arith_item *>(l.get()) > static_cast<arith_item *>(r.get()));
        }

        RBS_EXPORT id_expression::id_expression(const std::vector<rbs::id_token> &is) : rbs::ast::id_expression(is) {}
        std::vector<std::vector<expr>> id_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const { return ids.size() == 1 ? ctx.at(ids.at(0).id) : static_cast<fact *>(ctx.at(ids.at(0).id).get())->get(ids.at(1).id); }

        implication_expression::implication_expression(const rbs::ast::expression *const l, const rbs::ast::expression *const r) : rbs::ast::implication_expression(l, r) {}
        std::vector<std::vector<expr>> implication_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const
        {
            auto l = dynamic_cast<const ast::expression *>(left)->evaluate(ctx);
            auto r = dynamic_cast<const ast::expression *>(right)->evaluate(ctx);
            return std::make_shared<conjunction>({!l, r});
        }

        disjunction_expression::disjunction_expression(const std::vector<const rbs::ast::expression *> &es) : rbs::ast::disjunction_expression(es) {}
        std::vector<std::vector<expr>> disjunction_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const
        {
            std::vector<bool_expr> exprs;
            for (const auto &e : expressions)
                exprs.emplace_back(dynamic_cast<const ast::expression *>(e)->evaluate(scp, ctx));
            return scp.get_core().disj(exprs);
        }

        conjunction_expression::conjunction_expression(const std::vector<const rbs::ast::expression *> &es) : rbs::ast::conjunction_expression(es) {}
        std::vector<std::vector<expr>> conjunction_expression::evaluate(std::unordered_map<std::string, expr> &ctx) const
        {
            std::vector<bool_expr> exprs;
            for (const auto &e : expressions)
                exprs.emplace_back(dynamic_cast<const ast::expression *>(e)->evaluate(scp, ctx));
            return scp.get_core().conj(exprs);
        }
    } // namespace ast

    kb_parser::kb_parser(std::istream &is) : rbs::parser(is) {}
} // namespace kb
