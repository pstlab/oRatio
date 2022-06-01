#include "rule_parser.h"
#include <cassert>

namespace rbs
{
    using namespace ast;

    RBS_EXPORT parser::parser(std::istream &is) : lex(is) {}
    RBS_EXPORT parser::~parser() {}

    token *parser::next()
    {
        while (pos >= tks.size())
        {
            token *c_tk = lex.next();
            tks.emplace_back(c_tk);
        }
        return tks[pos++];
    }

    bool parser::match(const symbol &sym)
    {
        if (tk->sym == sym)
        {
            tk = next();
            return true;
        }
        else
            return false;
    }

    void parser::backtrack(const size_t &p) noexcept
    {
        pos = p;
        tk = tks[pos - 1];
    }

    RBS_EXPORT compilation_unit *parser::parse()
    {
        tk = next();

        std::vector<const statement *> ss;
        std::vector<const rule_declaration *> rs;

        while (tk->sym != EOF_ID)
            switch (tk->sym)
            {
            case ASSERT_ID:
            case RETRACT_ID:
            case ID_ID:
                ss.emplace_back(_statement());
                break;
            case LPAREN_ID:
                rs.emplace_back(_rule_declaration());
                break;
            default:
                error("expected either 'assert' or 'retract' or '(' or identifier..");
            }

        return new_compilation_unit(ss, rs);
    }

    ast::statement *parser::_statement()
    {
        switch (tk->sym)
        {
        case ASSERT_ID:
        {
            tk = next();
            std::vector<id_token> ns;
            std::vector<std::pair<const id_token, const expression *const>> assns;

            if (!match(ID_ID))
                error("expected identifier..");
            ns.emplace_back(*static_cast<id_token *>(tks[pos - 2]));

            if (match(COLON_ID))
            {
                if (!match(ID_ID))
                    error("expected identifier..");
                ns.emplace_back(*static_cast<id_token *>(tks[pos - 2]));
            }

            if (!match(LPAREN_ID))
                error("expected '('..");

            if (!match(RPAREN_ID))
            {
                do
                {
                    if (!match(ID_ID))
                        error("expected identifier..");
                    id_token assgn_name = *static_cast<id_token *>(tks[pos - 2]);

                    if (!match(COLON_ID))
                        error("expected ':'..");

                    expression *xpr = _expression();
                    assns.emplace_back(assgn_name, xpr);
                } while (match(COMMA_ID));

                if (!match(RPAREN_ID))
                    error("expected ')'..");
            }

            if (!match(SEMICOLON_ID))
                error("expected ';'..");

            return ns.size() == 2 ? new assert_statement(ns.at(0), ns.at(1), assns) : new assert_statement(id_token(0, 0, 0, 0, "f" + std::to_string(rand())), ns.at(0), assns);
        }
        case RETRACT_ID:
        {
            tk = next();
            std::vector<id_token> fns;

            do
            {
                if (!match(ID_ID))
                    error("expected identifier..");
                fns.emplace_back(*static_cast<id_token *>(tks[pos - 2]));
            } while (match(COMMA_ID));

            if (!match(SEMICOLON_ID))
                error("expected ';'..");

            return new retract_statement(fns);
        }
        case ID_ID:
        {
            id_token fn = *static_cast<id_token *>(tk);
            tk = next();
            switch (tk->sym)
            {
            case DOT_ID:
            { // assignment_statement..
                std::vector<std::vector<id_token>> ids;
                std::vector<const ast::expression *> xprs;
                tk = next();
                if (!match(ID_ID))
                    error("expected identifier..");
                ids.emplace_back(std::vector<id_token>({fn, *static_cast<id_token *>(tks[pos - 2])}));

                if (!match(EQ_ID))
                    error("expected '='..");

                xprs.emplace_back(_expression());

                while (match(COMMA_ID))
                {
                    std::vector<id_token> is;
                    if (!match(ID_ID))
                        error("expected identifier..");
                    is.emplace_back(*static_cast<id_token *>(tks[pos - 2]));

                    if (match(DOT_ID))
                    {
                        if (!match(ID_ID))
                            error("expected identifier..");
                        is.emplace_back(*static_cast<id_token *>(tks[pos - 2]));
                    }

                    if (!match(EQ_ID))
                        error("expected '='..");

                    xprs.emplace_back(_expression());
                }

                if (!match(SEMICOLON_ID))
                    error("expected ';'..");

                return new assignment_statement(ids, xprs);
            }
            case EQ_ID:
            { // assignment_statement..
                std::vector<std::vector<id_token>> ids;
                std::vector<const ast::expression *> xprs;
                tk = next();
                ids.emplace_back(std::vector<id_token>({fn}));

                xprs.emplace_back(_expression());

                while (match(COMMA_ID))
                {
                    std::vector<id_token> is;
                    if (!match(ID_ID))
                        error("expected identifier..");
                    is.emplace_back(*static_cast<id_token *>(tks[pos - 2]));

                    if (match(DOT_ID))
                    {
                        if (!match(ID_ID))
                            error("expected identifier..");
                        is.emplace_back(*static_cast<id_token *>(tks[pos - 2]));
                    }

                    if (!match(EQ_ID))
                        error("expected '='..");

                    xprs.emplace_back(_expression());
                }

                if (!match(SEMICOLON_ID))
                    error("expected ';'..");

                return new assignment_statement(ids, xprs);
            }
            case LPAREN_ID:
            { // function_statement..
                std::vector<const expression *> xprs;

                tk = next();
                if (!match(LPAREN_ID))
                    error("expected '('..");

                if (!match(RPAREN_ID))
                {
                    do
                    {
                        xprs.emplace_back(_expression());
                    } while (match(COMMA_ID));

                    if (!match(RPAREN_ID))
                        error("expected ')'..");
                }

                return new function_statement(fn, xprs);
            }
            default:
                error("expected either '.' or '=' or '('..");
                return nullptr;
            }
        }
        default:
            error("expected either 'assert' or 'retract' or identifier..");
            return nullptr;
        }
    }

    ast::rule_declaration *parser::_rule_declaration()
    {
        if (!match(LPAREN_ID))
            error("expected '('..");

        if (!match(ID_ID))
            error("expected identifier..");
        // the name of the rule..
        id_token n = *static_cast<id_token *>(tks[pos - 2]);

        expression *cond = _expression();
        std::vector<const statement *> ss;

        if (!match(IMPLICATION_ID))
            error("expected '=>'..");

        do
        {
            ss.emplace_back(_statement());
        } while (!match(RPAREN_ID));

        return new_rule_declaration(n, cond, ss);
    }

    expression *parser::_expression(const size_t &pr)
    {
        expression *e = nullptr;
        switch (tk->sym)
        {
        case BoolLiteral_ID:
            tk = next();
            e = new_bool_literal_expression(*static_cast<bool_token *>(tks[pos - 2]));
            break;
        case IntLiteral_ID:
            tk = next();
            e = new_int_literal_expression(*static_cast<int_token *>(tks[pos - 2]));
            break;
        case RealLiteral_ID:
            tk = next();
            e = new_real_literal_expression(*static_cast<real_token *>(tks[pos - 2]));
            break;
        case StringLiteral_ID:
            tk = next();
            e = new_string_literal_expression(*static_cast<string_token *>(tks[pos - 2]));
            break;
        case LPAREN_ID: // either a parenthesys expression or a cast..
        {
            tk = next();
            e = _expression();
            if (!match(RPAREN_ID))
                error("expected ')'..");
            break;
        }
        case PLUS_ID:
            tk = next();
            e = new_plus_expression(_expression(4));
            break;
        case MINUS_ID:
            tk = next();
            e = new_minus_expression(_expression(4));
            break;
        case BANG_ID:
            tk = next();
            e = new_not_expression(_expression(4));
            break;
        case ID_ID:
        {
            std::vector<id_token> is;
            is.emplace_back(*static_cast<id_token *>(tk));
            tk = next();

            switch (tk->sym)
            {
            case DOT_ID:
                tk = next();
                if (!match(ID_ID))
                    error("expected identifier..");
                is.emplace_back(*static_cast<id_token *>(tks[pos - 2]));
                e = new_id_expression(is);
                break;
            case COLON_ID:
            {
                tk = next();
                if (!match(ID_ID))
                    error("expected identifier..");
                is.emplace_back(*static_cast<id_token *>(tks[pos - 2]));
                std::vector<std::pair<const id_token, const expression *const>> assns;
                if (!match(LPAREN_ID))
                    error("expected '('..");

                if (!match(RPAREN_ID))
                {
                    do
                    {
                        if (!match(ID_ID))
                            error("expected identifier..");
                        id_token assgn_name = *static_cast<id_token *>(tks[pos - 2]);

                        if (!match(COLON_ID))
                            error("expected ':'..");

                        expression *xpr = _expression();
                        assns.emplace_back(assgn_name, xpr);
                    } while (match(COMMA_ID));

                    if (!match(RPAREN_ID))
                        error("expected ')'..");
                }
                e = new_fact_expression(is.at(0), is.at(1), assns);
            }
            break;
            case LPAREN_ID:
            {
                tk = next();
                std::vector<std::pair<const id_token, const expression *const>> assns;
                if (!match(RPAREN_ID))
                {
                    do
                    {
                        if (!match(ID_ID))
                            error("expected identifier..");
                        id_token assgn_name = *static_cast<id_token *>(tks[pos - 2]);

                        if (!match(COLON_ID))
                            error("expected ':'..");

                        expression *xpr = _expression();
                        assns.emplace_back(assgn_name, xpr);
                    } while (match(COMMA_ID));

                    if (!match(RPAREN_ID))
                        error("expected ')'..");
                }
                e = new_fact_expression(id_token(0, 0, 0, 0, "f" + std::to_string(rand())), is.at(0), assns);
            }
            break;
            default:
                error("expected either '.' or ':'..");
            }
            break;
        }
        default:
            error("expected either '(' or '+' or '-' or '!' or '[' or 'new' or a literal or an identifier..");
        }

        while (
            ((tk->sym == EQEQ_ID || tk->sym == BANGEQ_ID) && 0 >= pr) ||
            ((tk->sym == LT_ID || tk->sym == LTEQ_ID || tk->sym == GTEQ_ID || tk->sym == GT_ID || tk->sym == BAR_ID || tk->sym == AMP_ID) && 1 >= pr) ||
            ((tk->sym == PLUS_ID || tk->sym == MINUS_ID) && 2 >= pr) ||
            ((tk->sym == STAR_ID || tk->sym == SLASH_ID) && 3 >= pr))
        {
            switch (tk->sym)
            {
            case EQEQ_ID:
                assert(0 >= pr);
                tk = next();
                e = new_eq_expression(e, _expression(1));
                break;
            case BANGEQ_ID:
                assert(0 >= pr);
                tk = next();
                e = new_neq_expression(e, _expression(1));
                break;
            case LT_ID:
            {
                assert(1 >= pr);
                tk = next();
                expression *l = e;
                expression *r = _expression(2);
                e = new_lt_expression(l, r);
                break;
            }
            case LTEQ_ID:
            {
                assert(1 >= pr);
                tk = next();
                expression *l = e;
                expression *r = _expression(2);
                e = new_leq_expression(l, r);
                break;
            }
            case GTEQ_ID:
            {
                assert(1 >= pr);
                tk = next();
                expression *l = e;
                expression *r = _expression(2);
                e = new_geq_expression(l, r);
                break;
            }
            case GT_ID:
            {
                assert(1 >= pr);
                tk = next();
                expression *l = e;
                expression *r = _expression(2);
                e = new_gt_expression(l, r);
                break;
            }
            case BAR_ID:
            {
                assert(1 >= pr);
                std::vector<const expression *> xprs;
                xprs.emplace_back(e);

                while (match(BAR_ID))
                    xprs.emplace_back(_expression(2));

                e = new_disjunction_expression(xprs);
                break;
            }
            case AMP_ID:
            {
                assert(1 >= pr);
                std::vector<const expression *> xprs;
                xprs.emplace_back(e);

                while (match(AMP_ID))
                    xprs.emplace_back(_expression(2));

                e = new_conjunction_expression(xprs);
                break;
            }
            case PLUS_ID:
            {
                assert(2 >= pr);
                std::vector<const expression *> xprs;
                xprs.emplace_back(e);

                while (match(PLUS_ID))
                    xprs.emplace_back(_expression(3));

                e = new_addition_expression(xprs);
                break;
            }
            case MINUS_ID:
            {
                assert(2 >= pr);
                std::vector<const expression *> xprs;
                xprs.emplace_back(e);

                while (match(MINUS_ID))
                    xprs.emplace_back(_expression(3));

                e = new_subtraction_expression(xprs);
                break;
            }
            case STAR_ID:
            {
                assert(3 >= pr);
                std::vector<const expression *> xprs;
                xprs.emplace_back(e);

                while (match(STAR_ID))
                    xprs.emplace_back(_expression(4));

                e = new_multiplication_expression(xprs);
                break;
            }
            case SLASH_ID:
            {
                assert(3 >= pr);
                std::vector<const expression *> xprs;
                xprs.emplace_back(e);

                while (match(SLASH_ID))
                    xprs.emplace_back(_expression(4));

                e = new_division_expression(xprs);
                break;
            }
            default:
                error("expected either '==' or '!=' or '<' or '<=' or '>' or '>=' or '->' or '|' or '&' or '+' or '-' or '*' or '/'..");
            }
        }

        return e;
    }

    void parser::error(const std::string &err) { throw std::invalid_argument("[" + std::to_string(tk->start_line + 1) + ", " + std::to_string(tk->start_pos + 1) + "] " + err); }
} // namespace rbs