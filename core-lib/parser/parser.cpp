#include "parser.h"
#include "lexer.h"
#include "type_declaration.h"
#include "method_declaration.h"
#include "predicate_declaration.h"
#include "typedef_declaration.h"
#include "enum_declaration.h"
#include "class_declaration.h"
#include "variable_declaration.h"
#include "field_declaration.h"
#include "constructor_declaration.h"
#include "compilation_unit.h"
#include "local_field_statement.h"
#include "assignment_statement.h"
#include "expression_statement.h"
#include "disjunction_statement.h"
#include "conjunction_statement.h"
#include "formula_statement.h"
#include "return_statement.h"
#include "bool_literal_expression.h"
#include "int_literal_expression.h"
#include "real_literal_expression.h"
#include "string_literal_expression.h"
#include "cast_expression.h"
#include "plus_expression.h"
#include "minus_expression.h"
#include "not_expression.h"
#include "range_expression.h"
#include "constructor_expression.h"
#include "eq_expression.h"
#include "neq_expression.h"
#include "lt_expression.h"
#include "leq_expression.h"
#include "geq_expression.h"
#include "gt_expression.h"
#include "function_expression.h"
#include "id_expression.h"
#include "implication_expression.h"
#include "disjunction_expression.h"
#include "conjunction_expression.h"
#include "addition_expression.h"
#include "subtraction_expression.h"
#include "multiplication_expression.h"
#include "division_expression.h"
#include <cassert>

namespace ratio
{

using namespace ast;

parser::parser(std::istream &is) : lex(is) {}
parser::~parser() {}

token *parser::next()
{
    while (pos >= tks.size())
    {
        token *c_tk = lex.next();
        tks.push_back(c_tk);
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

void parser::backtrack(const size_t &p)
{
    pos = p;
    tk = tks[pos - 1];
}

compilation_unit *parser::parse()
{
    tk = next();

    std::vector<type_declaration *> ts;
    std::vector<method_declaration *> ms;
    std::vector<predicate_declaration *> ps;
    std::vector<statement *> ss;

    while (tk->sym != EOF_ID)
    {
        switch (tk->sym)
        {
        case TYPEDEF_ID:
            ts.push_back(_typedef_declaration());
            break;
        case ENUM_ID:
            ts.push_back(_enum_declaration());
            break;
        case CLASS_ID:
            ts.push_back(_class_declaration());
            break;
        case PREDICATE_ID:
            ps.push_back(_predicate_declaration());
            break;
        case VOID_ID:
            ms.push_back(_method_declaration());
            break;
        case BOOL_ID:
        case INT_ID:
        case REAL_ID:
        case STRING_ID:
        case LBRACE_ID:
        case BANG_ID:
        case FACT_ID:
        case GOAL_ID:
            ss.push_back(_statement());
            break;
        case ID_ID:
        {
            size_t c_pos = pos;
            tk = next();
            while (match(DOT_ID))
            {
                if (!match(ID_ID))
                    error("expected identifier..");
            }
            if (match(ID_ID) && match(LPAREN_ID))
            {
                backtrack(c_pos);
                ms.push_back(_method_declaration());
            }
            else
            {
                backtrack(c_pos);
                ss.push_back(_statement());
            }
            break;
        }
        default:
            error("expected either 'typedef' or 'enum' or 'class' or 'predicate' or 'void' or identifier..");
        }
    }

    return new compilation_unit(ms, ps, ts, ss);
}

typedef_declaration *parser::_typedef_declaration()
{
    std::string n;
    std::string pt;
    expression *xpr;

    if (!match(TYPEDEF_ID))
        error("expected 'typedef'..");

    switch (tk->sym)
    {
    case BOOL_ID:
        pt = "bool";
        break;
    case INT_ID:
        pt = "int";
        break;
    case REAL_ID:
        pt = "real";
        break;
    case STRING_ID:
        pt = "string";
        break;
    default:
        error("expected primitive type..");
    }
    tk = next();

    xpr = _expression();

    if (!match(ID_ID))
        error("expected identifier..");
    n = static_cast<id_token *>(tks[pos - 2])->id;

    if (!match(SEMICOLON_ID))
        error("expected ';'..");

    return new typedef_declaration(n, pt, xpr);
}

enum_declaration *parser::_enum_declaration()
{
    std::string n;
    std::vector<std::string> es;
    std::vector<std::vector<std::string>> trs;

    if (!match(ENUM_ID))
        error("expected 'enum'..");

    if (!match(ID_ID))
        error("expected identifier..");
    n = static_cast<id_token *>(tks[pos - 2])->id;

    do
    {
        switch (tk->sym)
        {
        case LBRACE_ID:
            tk = next();
            if (!match(StringLiteral_ID))
                error("expected string literal..");
            es.push_back(static_cast<string_token *>(tks[pos - 2])->str);

            while (match(COMMA_ID))
            {
                if (!match(StringLiteral_ID))
                    error("expected string literal..");
                es.push_back(static_cast<string_token *>(tks[pos - 2])->str);
            }

            if (!match(RBRACE_ID))
                error("expected '}'..");
            break;
        case ID_ID:
        {
            std::vector<std::string> ids;
            ids.push_back(static_cast<id_token *>(tk)->id);
            tk = next();
            while (match(DOT_ID))
            {
                if (!match(ID_ID))
                    error("expected identifier..");
                ids.push_back(static_cast<id_token *>(tks[pos - 2])->id);
            }
            trs.push_back(ids);
            break;
        }
        default:
            error("expected either '{' or identifier..");
        }
    } while (match(BAR_ID));

    if (!match(SEMICOLON_ID))
        error("expected ';'..");

    return new enum_declaration(n, es, trs);
}

class_declaration *parser::_class_declaration()
{
    std::string n;                             // the name of the class..
    std::vector<std::vector<std::string>> bcs; // the base classes..
    std::vector<field_declaration *> fs;       // the fields of the class..
    std::vector<constructor_declaration *> cs; // the constructors of the class..
    std::vector<method_declaration *> ms;      // the methods of the class..
    std::vector<predicate_declaration *> ps;   // the predicates of the class..
    std::vector<type_declaration *> ts;        // the types of the class..

    if (!match(CLASS_ID))
        error("expected 'class'..");

    if (!match(ID_ID))
        error("expected identifier..");
    n = static_cast<id_token *>(tks[pos - 2])->id;

    if (match(COLON_ID))
    {
        do
        {
            std::vector<std::string> ids;
            do
            {
                if (!match(ID_ID))
                    error("expected identifier..");
                ids.push_back(static_cast<id_token *>(tks[pos - 2])->id);
            } while (match(DOT_ID));
            bcs.push_back(ids);
        } while (match(COMMA_ID));
    }

    if (!match(LBRACE_ID))
        error("expected '{'..");

    while (!match(RBRACE_ID))
    {
        switch (tk->sym)
        {
        case TYPEDEF_ID:
            ts.push_back(_typedef_declaration());
            break;
        case ENUM_ID:
            ts.push_back(_enum_declaration());
            break;
        case CLASS_ID:
            ts.push_back(_class_declaration());
            break;
        case PREDICATE_ID:
            ps.push_back(_predicate_declaration());
            break;
        case VOID_ID:
            ms.push_back(_method_declaration());
            break;
        case BOOL_ID:
        case INT_ID:
        case REAL_ID:
        case STRING_ID: // either a primitive type method or a field declaration..
        {
            size_t c_pos = pos;
            tk = next();
            if (!match(ID_ID))
                error("expected identifier..");
            switch (tk->sym)
            {
            case LPAREN_ID:
                backtrack(c_pos);
                ms.push_back(_method_declaration());
                break;
            case EQ_ID:
            case SEMICOLON_ID:
                backtrack(c_pos);
                fs.push_back(_field_declaration());
                break;
            default:
                error("expected either '(' or '=' or ';'..");
            }
            break;
        }
        case ID_ID: // either a constructor, a method or a field declaration..
        {
            size_t c_pos = pos;
            tk = next();
            switch (tk->sym)
            {
            case LPAREN_ID:
                backtrack(c_pos);
                cs.push_back(_constructor_declaration());
                break;
            case DOT_ID:
                while (match(DOT_ID))
                    if (!match(ID_ID))
                        error("expected identifier..");
                if (!match(ID_ID))
                    error("expected identifier..");
                switch (tk->sym)
                {
                case LPAREN_ID:
                    backtrack(c_pos);
                    ms.push_back(_method_declaration());
                    break;
                case EQ_ID:
                case SEMICOLON_ID:
                    backtrack(c_pos);
                    fs.push_back(_field_declaration());
                    break;
                default:
                    error("expected either '(' or '=' or ';'..");
                }
                break;
            case ID_ID:
                tk = next();
                switch (tk->sym)
                {
                case LPAREN_ID:
                    backtrack(c_pos);
                    ms.push_back(_method_declaration());
                    break;
                case EQ_ID:
                case SEMICOLON_ID:
                    backtrack(c_pos);
                    fs.push_back(_field_declaration());
                    break;
                default:
                    error("expected either '(' or '=' or ';'..");
                }
                break;
            default:
                error("expected either '(' or '.' or an identifier..");
            }
            break;
        }
        default:
            error("expected either 'typedef' or 'enum' or 'class' or 'predicate' or 'void' or identifier..");
        }
    }

    return new class_declaration(n, bcs, fs, cs, ms, ps, ts);
}

field_declaration *parser::_field_declaration()
{
    std::vector<std::string> ids;
    std::string n;
    std::vector<variable_declaration *> ds;

    switch (tk->sym)
    {
    case BOOL_ID:
        ids.push_back("bool");
        tk = next();
        break;
    case INT_ID:
        ids.push_back("int");
        tk = next();
        break;
    case REAL_ID:
        ids.push_back("real");
        tk = next();
        break;
    case STRING_ID:
        ids.push_back("string");
        tk = next();
        break;
    case ID_ID:
        ids.push_back(static_cast<id_token *>(tk)->id);
        tk = next();
        while (match(DOT_ID))
        {
            if (!match(ID_ID))
                error("expected identifier..");
            ids.push_back(static_cast<id_token *>(tks[pos - 2])->id);
        }
        break;
    default:
        error("expected either 'bool' or 'int' or 'real' or 'string' or an identifier..");
    }

    if (!match(ID_ID))
        error("expected identifier..");
    n = static_cast<id_token *>(tks[pos - 2])->id;

    if (match(EQ_ID))
        ds.push_back(new variable_declaration(n, _expression()));
    else
        ds.push_back(new variable_declaration(n));

    while (match(COMMA_ID))
    {
        if (!match(ID_ID))
            error("expected identifier..");
        n = static_cast<id_token *>(tks[pos - 2])->id;

        if (match(EQ_ID))
            ds.push_back(new variable_declaration(n, _expression()));
        else
            ds.push_back(new variable_declaration(n));
    }

    if (!match(SEMICOLON_ID))
        error("expected ';'..");

    return new field_declaration(ids, ds);
}

method_declaration *parser::_method_declaration()
{
    std::vector<std::string> ids;
    std::string n;
    std::vector<std::pair<std::vector<std::string>, std::string>> pars;
    std::vector<statement *> stmnts;

    if (!match(VOID_ID))
    {
        do
        {
            if (!match(ID_ID))
                error("expected identifier..");
            ids.push_back(static_cast<id_token *>(tks[pos - 2])->id);
        } while (match(DOT_ID));
    }

    if (!match(ID_ID))
        error("expected identifier..");
    n = static_cast<id_token *>(tks[pos - 2])->id;

    if (!match(LPAREN_ID))
        error("expected '('..");

    if (!match(RPAREN_ID))
    {
        do
        {
            std::vector<std::string> p_ids;
            switch (tk->sym)
            {
            case BOOL_ID:
                p_ids.push_back("bool");
                tk = next();
                break;
            case INT_ID:
                p_ids.push_back("int");
                tk = next();
                break;
            case REAL_ID:
                p_ids.push_back("real");
                tk = next();
                break;
            case STRING_ID:
                p_ids.push_back("string");
                tk = next();
                break;
            case ID_ID:
                p_ids.push_back(static_cast<id_token *>(tk)->id);
                tk = next();
                while (match(DOT_ID))
                {
                    if (!match(ID_ID))
                        error("expected identifier..");
                    p_ids.push_back(static_cast<id_token *>(tks[pos - 2])->id);
                }
                break;
            default:
                error("expected either 'bool' or 'int' or 'real' or 'string' or an identifier..");
            }
            if (!match(ID_ID))
                error("expected identifier..");
            std::string pn = static_cast<id_token *>(tks[pos - 2])->id;
            pars.push_back({p_ids, pn});
        } while (match(COMMA_ID));

        if (!match(RPAREN_ID))
            error("expected ')'..");
    }

    if (!match(LBRACE_ID))
        error("expected '{'..");

    while (!match(RBRACE_ID))
        stmnts.push_back(_statement());

    return new method_declaration(ids, n, pars, stmnts);
}

constructor_declaration *parser::_constructor_declaration()
{
    std::vector<std::pair<std::vector<std::string>, std::string>> pars;
    std::vector<std::pair<std::string, std::vector<expression *>>> il;
    std::vector<statement *> stmnts;

    if (!match(ID_ID))
        error("expected identifier..");

    if (!match(LPAREN_ID))
        error("expected '('..");

    if (!match(RPAREN_ID))
    {
        do
        {
            std::vector<std::string> p_ids;
            switch (tk->sym)
            {
            case ID_ID:
                p_ids.push_back(static_cast<id_token *>(tk)->id);
                tk = next();
                while (match(DOT_ID))
                {
                    if (!match(ID_ID))
                        error("expected identifier..");
                    p_ids.push_back(static_cast<id_token *>(tks[pos - 2])->id);
                }
                break;
            case BOOL_ID:
                p_ids.push_back("bool");
                tk = next();
                break;
            case INT_ID:
                p_ids.push_back("int");
                tk = next();
                break;
            case REAL_ID:
                p_ids.push_back("real");
                tk = next();
                break;
            case STRING_ID:
                p_ids.push_back("string");
                tk = next();
                break;
            }
            if (!match(ID_ID))
                error("expected identifier..");
            std::string pn = static_cast<id_token *>(tks[pos - 2])->id;
            pars.push_back({p_ids, pn});
        } while (match(COMMA_ID));

        if (!match(RPAREN_ID))
            error("expected ')'..");
    }

    if (match(COLON_ID))
    {
        do
        {
            std::string pn;
            std::vector<expression *> xprs;
            if (!match(ID_ID))
                error("expected identifier..");
            pn = static_cast<id_token *>(tks[pos - 2])->id;

            if (!match(LPAREN_ID))
                error("expected '('..");

            if (!match(RPAREN_ID))
            {
                do
                {
                    xprs.push_back(_expression());
                } while (match(COMMA_ID));

                if (!match(RPAREN_ID))
                    error("expected ')'..");
            }
            il.push_back({pn, xprs});
        } while (match(COMMA_ID));
    }

    if (!match(LBRACE_ID))
        error("expected '{'..");

    while (!match(RBRACE_ID))
        stmnts.push_back(_statement());

    return new constructor_declaration(pars, il, stmnts);
}

predicate_declaration *parser::_predicate_declaration()
{
    std::string n;
    std::vector<std::pair<std::vector<std::string>, std::string>> pars;
    std::vector<std::vector<std::string>> pl;
    std::vector<statement *> stmnts;

    if (!match(PREDICATE_ID))
        error("expected 'predicate'..");

    if (!match(ID_ID))
        error("expected identifier..");
    n = static_cast<id_token *>(tks[pos - 2])->id;

    if (!match(LPAREN_ID))
        error("expected '('..");

    if (!match(RPAREN_ID))
    {
        do
        {
            std::vector<std::string> p_ids;
            switch (tk->sym)
            {
            case BOOL_ID:
                p_ids.push_back("bool");
                tk = next();
                break;
            case INT_ID:
                p_ids.push_back("int");
                tk = next();
                break;
            case REAL_ID:
                p_ids.push_back("real");
                tk = next();
                break;
            case STRING_ID:
                p_ids.push_back("string");
                tk = next();
                break;
            case ID_ID:
                p_ids.push_back(static_cast<id_token *>(tk)->id);
                tk = next();
                while (match(DOT_ID))
                {
                    if (!match(ID_ID))
                        error("expected identifier..");
                    p_ids.push_back(static_cast<id_token *>(tks[pos - 2])->id);
                }
                break;
            }
            if (!match(ID_ID))
                error("expected identifier..");
            std::string pn = static_cast<id_token *>(tks[pos - 2])->id;
            pars.push_back({p_ids, pn});
        } while (match(COMMA_ID));

        if (!match(RPAREN_ID))
            error("expected ')'..");
    }

    if (match(COLON_ID))
    {
        do
        {
            std::vector<std::string> p_ids;
            do
            {
                if (!match(ID_ID))
                    error("expected identifier..");
                p_ids.push_back(static_cast<id_token *>(tks[pos - 2])->id);
            } while (match(DOT_ID));
            pl.push_back(p_ids);
        } while (match(COMMA_ID));
    }

    if (!match(LBRACE_ID))
        error("expected '{'..");

    while (!match(RBRACE_ID))
        stmnts.push_back(_statement());

    return new predicate_declaration(n, pars, pl, stmnts);
}

statement *parser::_statement()
{
    switch (tk->sym)
    {
    case BOOL_ID:
    case INT_ID:
    case REAL_ID:
    case STRING_ID: // a local field having a primitive type..
    {
        std::vector<std::string> ids;
        switch (tk->sym)
        {
        case BOOL_ID:
            ids.push_back("bool");
            break;
        case INT_ID:
            ids.push_back("int");
            break;
        case REAL_ID:
            ids.push_back("real");
            break;
        case STRING_ID:
            ids.push_back("string");
            break;
        }
        tk = next();

        if (!match(ID_ID))
            error("expected identifier..");
        std::string n = static_cast<id_token *>(tks[pos - 2])->id;

        expression *e = nullptr;
        if (tk->sym == EQ_ID)
        {
            tk = next();
            e = _expression();
        }

        if (!match(SEMICOLON_ID))
            error("expected ';'..");

        return new local_field_statement(ids, n, e);
    }
    case ID_ID: // either a local field, an assignment or an expression..
    {
        size_t c_pos = pos;
        std::vector<std::string> ids;
        ids.push_back(static_cast<id_token *>(tk)->id);
        tk = next();
        while (match(DOT_ID))
        {
            if (!match(ID_ID))
                error("expected identifier..");
            ids.push_back(static_cast<id_token *>(tks[pos - 2])->id);
        }

        switch (tk->sym)
        {
        case ID_ID: // a local field..
        {
            std::string n = static_cast<id_token *>(tk)->id;
            expression *e = nullptr;
            tk = next();
            if (tk->sym == EQ_ID)
            {
                tk = next();
                e = _expression();
            }

            if (!match(SEMICOLON_ID))
                error("expected ';'..");

            return new local_field_statement(ids, n, e);
        }
        case EQ_ID: // an assignment..
        {
            std::string id = ids.back();
            ids.pop_back();
            tk = next();
            expression *xpr = _expression();
            if (!match(SEMICOLON_ID))
                error("expected ';'..");
            return new assignment_statement(ids, id, xpr);
        }
        case PLUS_ID: // an expression..
        case MINUS_ID:
        case STAR_ID:
        case SLASH_ID:
        case LT_ID:
        case LTEQ_ID:
        case EQEQ_ID:
        case GTEQ_ID:
        case GT_ID:
        case BANGEQ_ID:
        case IMPLICATION_ID:
        case BAR_ID:
        case AMP_ID:
        case CARET_ID:
        {
            backtrack(c_pos);
            expression *xpr = _expression();
            if (!match(SEMICOLON_ID))
                error("expected ';'..");
            return new expression_statement(xpr);
        }
        default:
            error("expected either '=' or an identifier..");
        }
    }
    case LBRACE_ID: // either a block or a disjunction..
    {
        tk = next();
        std::vector<const statement *> stmnts;
        do
        {
            stmnts.push_back(_statement());
        } while (!match(RBRACE_ID));
        switch (tk->sym)
        {
        case LBRACKET_ID:
        case OR_ID: // a disjunctive statement..
        {
            std::vector<std::pair<std::vector<const statement *>, const expression *const>> disjs;
            expression *e = nullptr;
            if (match(LBRACKET_ID))
            {
                e = _expression();
                if (!match(RBRACKET_ID))
                    error("expected ']'..");
            }
            disjs.push_back({stmnts, e});
            while (match(OR_ID))
            {
                stmnts.clear();
                e = nullptr;
                if (!match(LBRACE_ID))
                    error("expected '{'..");
                do
                {
                    stmnts.push_back(_statement());
                } while (!match(RBRACE_ID));
                if (match(LBRACKET_ID))
                {
                    e = _expression();
                    if (!match(RBRACKET_ID))
                        error("expected ']'..");
                }
                disjs.push_back({stmnts, e});
            }
            return new disjunction_statement(disjs);
        }
        default: // a conjunction statement..
            return new conjunction_statement(stmnts);
        }
    }
    case FACT_ID:
    case GOAL_ID:
    {
        bool isf = tk->sym == FACT_ID;
        tk = next();
        std::string fn;
        std::vector<std::string> scp;
        std::string pn;
        std::vector<std::pair<std::string, const expression *>> assgns;

        if (!match(ID_ID))
            error("expected identifier..");
        fn = static_cast<id_token *>(tks[pos - 2])->id;

        if (!match(EQ_ID))
            error("expected '='..");

        if (!match(NEW_ID))
            error("expected 'new'..");

        do
        {
            if (!match(ID_ID))
                error("expected identifier..");
            scp.push_back(static_cast<id_token *>(tks[pos - 2])->id);
        } while (match(DOT_ID));

        pn = scp.back();
        scp.pop_back();

        if (!match(LPAREN_ID))
            error("expected '('..");

        if (!match(RPAREN_ID))
        {
            do
            {
                if (!match(ID_ID))
                    error("expected identifier..");
                std::string assgn_name = static_cast<id_token *>(tks[pos - 2])->id;

                if (!match(COLON_ID))
                    error("expected ':'..");

                expression *xpr = _expression();
                assgns.push_back({assgn_name, xpr});
            } while (match(COMMA_ID));

            if (!match(RPAREN_ID))
                error("expected ')'..");
        }

        if (!match(SEMICOLON_ID))
            error("expected ';'..");
        return new formula_statement(isf, fn, scp, pn, assgns);
    }
    case RETURN_ID:
    {
        expression *xpr = _expression();
        if (!match(SEMICOLON_ID))
            error("expected ';'..");
        return new return_statement(xpr);
    }
    default:
    {
        expression *xpr = _expression();
        if (!match(SEMICOLON_ID))
            error("expected ';'..");
        return new expression_statement(xpr);
    }
    }
}

expression *parser::_expression(const size_t &pr)
{
    expression *e = nullptr;
    switch (tk->sym)
    {
    case TRUE_ID:
    case FALSE_ID:
        tk = next();
        e = new bool_literal_expression(tks[pos - 2]->sym == TRUE_ID);
        break;
    case IntLiteral_ID:
        tk = next();
        e = new int_literal_expression(static_cast<int_token *>(tks[pos - 2])->val);
        break;
    case RealLiteral_ID:
        tk = next();
        e = new real_literal_expression(static_cast<real_token *>(tks[pos - 2])->val);
        break;
    case StringLiteral_ID:
        tk = next();
        e = new string_literal_expression(static_cast<string_token *>(tks[pos - 2])->str);
        break;
    case LPAREN_ID: // either a parenthesys expression or a cast..
    {
        tk = next();

        size_t c_pos = pos;
        do
        {
            if (!match(ID_ID))
                error("expected identifier..");
        } while (match(DOT_ID));

        if (match(RPAREN_ID)) // a cast..
        {
            backtrack(c_pos);
            std::vector<std::string> ids;
            do
            {
                if (!match(ID_ID))
                    error("expected identifier..");
                ids.push_back(static_cast<id_token *>(tks[pos - 2])->id);
            } while (match(DOT_ID));

            if (!match(RPAREN_ID))
                error("expected ')'..");
            expression *xpr = _expression();
            e = new cast_expression(ids, xpr);
        }
        else // a parenthesis..
        {
            backtrack(c_pos);
            expression *xpr = _expression();
            if (!match(RPAREN_ID))
                error("expected ')'..");
            e = xpr;
        }
        break;
    }
    case PLUS_ID:
        tk = next();
        e = new plus_expression(_expression(4));
        break;
    case MINUS_ID:
        tk = next();
        e = new minus_expression(_expression(4));
        break;
    case BANG_ID:
        tk = next();
        e = new not_expression(_expression(4));
        break;
    case LBRACKET_ID:
    {
        tk = next();
        expression *min_e = _expression();
        expression *max_e = _expression();

        if (!match(RBRACKET_ID))
            error("expected ']'..");

        e = new range_expression(min_e, max_e);
        break;
    }
    case NEW_ID:
    {
        tk = next();
        std::vector<std::string> ids;
        do
        {
            if (!match(ID_ID))
                error("expected identifier..");
            ids.push_back(static_cast<id_token *>(tks[pos - 2])->id);
        } while (match(DOT_ID));

        std::vector<expression *> xprs;
        if (!match(LPAREN_ID))
            error("expected '('..");

        if (!match(RPAREN_ID))
        {
            do
            {
                xprs.push_back(_expression());
            } while (match(COMMA_ID));

            if (!match(RPAREN_ID))
                error("expected ')'..");
        }

        e = new constructor_expression(ids, xprs);
        break;
    }
    case ID_ID:
    {
        std::vector<std::string> is;
        is.push_back(static_cast<id_token *>(tk)->id);
        tk = next();
        while (match(DOT_ID))
        {
            if (!match(ID_ID))
                error("expected identifier..");
            is.push_back(static_cast<id_token *>(tks[pos - 2])->id);
        }
        if (match(LPAREN_ID))
        {
            tk = next();
            std::string fn = is.back();
            is.pop_back();
            std::vector<expression *> xprs;
            if (!match(LPAREN_ID))
                error("expected '('..");

            if (!match(RPAREN_ID))
            {
                do
                {
                    xprs.push_back(_expression());
                } while (match(COMMA_ID));

                if (!match(RPAREN_ID))
                    error("expected ')'..");
            }

            e = new function_expression(is, fn, xprs);
        }
        else
            e = new id_expression(is);
        break;
    }
    default:
        error("expected either '(' or '+' or '-' or '!' or '[' or 'new' or a literal or an identifier..");
    }

    while (
        ((tk->sym == EQEQ_ID || tk->sym == BANGEQ_ID) && 0 >= pr) ||
        ((tk->sym == LT_ID || tk->sym == LTEQ_ID || tk->sym == GTEQ_ID || tk->sym == GT_ID || tk->sym == IMPLICATION_ID || tk->sym == BAR_ID || tk->sym == AMP_ID || tk->sym == CARET_ID) && 1 >= pr) ||
        ((tk->sym == PLUS_ID || tk->sym == MINUS_ID) && 2 >= pr) ||
        ((tk->sym == STAR_ID || tk->sym == SLASH_ID) && 3 >= pr))
    {
        switch (tk->sym)
        {
        case EQEQ_ID:
            assert(0 >= pr);
            tk = next();
            e = new eq_expression(e, _expression(1));
            break;
        case BANGEQ_ID:
            assert(0 >= pr);
            tk = next();
            e = new neq_expression(e, _expression(1));
            break;
        case LT_ID:
        {
            assert(1 >= pr);
            tk = next();
            expression *l = e;
            expression *r = _expression(2);
            e = new lt_expression(l, r);
            break;
        }
        case LTEQ_ID:
        {
            assert(1 >= pr);
            tk = next();
            expression *l = e;
            expression *r = _expression(2);
            e = new leq_expression(l, r);
            break;
        }
        case GTEQ_ID:
        {
            assert(1 >= pr);
            tk = next();
            expression *l = e;
            expression *r = _expression(2);
            e = new geq_expression(l, r);
            break;
        }
        case GT_ID:
        {
            assert(1 >= pr);
            tk = next();
            expression *l = e;
            expression *r = _expression(2);
            e = new gt_expression(l, r);
            break;
        }
        case IMPLICATION_ID:
        {
            assert(1 >= pr);
            tk = next();
            expression *l = e;
            expression *r = _expression(2);
            e = new implication_expression(l, r);
            break;
        }
        case BAR_ID:
        {
            assert(1 >= pr);
            std::vector<expression *> xprs;
            xprs.push_back(e);

            while (match(BAR_ID))
                xprs.push_back(_expression(2));

            e = new disjunction_expression(xprs);
            break;
        }
        case AMP_ID:
        {
            assert(1 >= pr);
            std::vector<expression *> xprs;
            xprs.push_back(e);

            while (match(BAR_ID))
                xprs.push_back(_expression(2));

            e = new conjunction_expression(xprs);
            break;
        }
        case CARET_ID:
        {
            assert(1 >= pr);
            std::vector<expression *> xprs;
            xprs.push_back(e);

            while (match(BAR_ID))
                xprs.push_back(_expression(2));

            e = new exct_one_expression(xprs);
            break;
        }
        case PLUS_ID:
        {
            assert(2 >= pr);
            std::vector<expression *> xprs;
            xprs.push_back(e);

            while (match(PLUS_ID))
                xprs.push_back(_expression(3));

            e = new addition_expression(xprs);
            break;
        }
        case MINUS_ID:
        {
            assert(2 >= pr);
            std::vector<expression *> xprs;
            xprs.push_back(e);

            while (match(MINUS_ID))
                xprs.push_back(_expression(3));

            e = new subtraction_expression(xprs);
            break;
        }
        case STAR_ID:
        {
            assert(3 >= pr);
            std::vector<expression *> xprs;
            xprs.push_back(e);

            while (match(STAR_ID))
                xprs.push_back(_expression(4));

            e = new multiplication_expression(xprs);
            break;
        }
        case SLASH_ID:
        {
            assert(3 >= pr);
            std::vector<expression *> xprs;
            xprs.push_back(e);

            while (match(SLASH_ID))
                xprs.push_back(_expression(4));

            e = new division_expression(xprs);
            break;
        }
        }
    }

    return e;
}

void parser::error(const std::string &err) { throw std::invalid_argument("[" + std::to_string(tk->start_line) + ", " + std::to_string(tk->start_pos) + "] " + err); }
}