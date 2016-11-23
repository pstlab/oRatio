/*
 * Copyright (C) 2016 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "expression_visitor.h"
#include "parser.h"
#include "../core.h"
#include "../item.h"
#include "../type.h"
#include "../method.h"
#include "../constructor.h"
#include "type_visitor.h"

using namespace oratio;

expression_visitor::expression_visitor(parser * const p, env * const e) : _parser(p), _env(e) { }

expression_visitor::~expression_visitor() { }

antlrcpp::Any expression_visitor::visitLiteral_expression(oRatioParser::Literal_expressionContext* ctx) {
    if (ctx->literal()->numeric) {
        if (ctx->literal()->numeric->getText().find('.') != ctx->literal()->numeric->getText().npos) {
            return static_cast<item*> (_parser->_core->new_real(std::stod(ctx->literal()->numeric->getText())));
        }
        else {
            return static_cast<item*> (_parser->_core->new_int(std::stol(ctx->literal()->numeric->getText())));
        }
    }
    else if (ctx->literal()->string) {
        std::string val = ctx->literal()->string->getText();
        val = val.substr(1, val.size() - 2);
        return static_cast<item*> (_parser->_core->new_string(val));
    }
    else if (ctx->literal()->t) {
        return static_cast<item*> (_parser->_core->new_bool(true));
    }
    else if (ctx->literal()->f) {
        return static_cast<item*> (_parser->_core->new_bool(false));
    }
    else {
        return nullptr;
    }
}

antlrcpp::Any expression_visitor::visitParentheses_expression(oRatioParser::Parentheses_expressionContext* ctx) {
    return visit(ctx->expr());
}

antlrcpp::Any expression_visitor::visitMultiplication_expression(oRatioParser::Multiplication_expressionContext* ctx) {
    std::vector<arith_item*> is;
    for (const auto& e : ctx->expr()) {
        is.push_back(visit(e).as<arith_item*>());
    }
    return _parser->_core->mult(is);
}

antlrcpp::Any expression_visitor::visitDivision_expression(oRatioParser::Division_expressionContext* ctx) {
    return _parser->_core->div(visit(ctx->expr(0)).as<arith_item*>(), visit(ctx->expr(1)).as<arith_item*>());
}

antlrcpp::Any expression_visitor::visitAddition_expression(oRatioParser::Addition_expressionContext* ctx) {
    std::vector<arith_item*> is;
    for (const auto& e : ctx->expr()) {
        is.push_back(visit(e).as<arith_item*>());
    }
    return _parser->_core->sum(is);
}

antlrcpp::Any expression_visitor::visitSubtraction_expression(oRatioParser::Subtraction_expressionContext* ctx) {
    std::vector<arith_item*> is;
    for (const auto& e : ctx->expr()) {
        is.push_back(visit(e).as<arith_item*>());
    }
    return _parser->_core->sub(is);
}

antlrcpp::Any expression_visitor::visitMinus_expression(oRatioParser::Minus_expressionContext* ctx) {
    return _parser->_core->minus(visit(ctx->expr()).as<arith_item*>());
}

antlrcpp::Any expression_visitor::visitNot_expression(oRatioParser::Not_expressionContext* ctx) {
    return _parser->_core->negate(visit(ctx->expr()).as<bool_item*>());
}

antlrcpp::Any expression_visitor::visitQualified_id_expression(oRatioParser::Qualified_id_expressionContext* ctx) {
    env * c_env = _env;
    if (ctx->qualified_id()->t) {
        c_env = c_env->get(THIS_KEYWORD);
    }
    for (const auto& id : ctx->qualified_id()->ID()) {
        c_env = c_env->get(id->getText());
        if (!c_env) {
            _parser->_parser->notifyErrorListeners(id->getSymbol(), "cannot find symbol..", nullptr);
            return nullptr;
        }
    }
    return dynamic_cast<item*> (c_env);
}

antlrcpp::Any expression_visitor::visitFunction_expression(oRatioParser::Function_expressionContext* ctx) {
    env * c_env = _env;
    method * m;

    std::vector<item*> exprs;
    std::vector<const type*> par_types;
    if (ctx->expr_list()) {
        for (const auto& expr : ctx->expr_list()->expr()) {
            item * i = expression_visitor(_parser, _env).visit(expr).as<item*>();
            exprs.push_back(i);
            par_types.push_back(i->get_type());
        }
    }

    if (ctx->object) {
        if (ctx->object->t) {
            c_env = c_env->get(THIS_KEYWORD);
        }
        for (const auto& id : ctx->object->ID()) {
            c_env = c_env->get(id->getText());
            if (!c_env) {
                _parser->_parser->notifyErrorListeners(id->getSymbol(), "cannot find symbol..", nullptr);
                return nullptr;
            }
        }
        m = dynamic_cast<item*> (c_env)->get_type()->get_method(ctx->function_name->getText(), par_types);
    }
    else {
        m = _parser->_scopes.at(ctx)->get_method(ctx->function_name->getText(), par_types);
    }

    if (!m) {
        _parser->_parser->notifyErrorListeners(ctx->function_name, "cannot find method..", nullptr);
        return nullptr;
    }
    bool invoke = m->invoke(c_env, exprs);
    assert(invoke && "functions should not create inconsistencies..");
    return c_env->get(RETURN_KEYWORD);
}

antlrcpp::Any expression_visitor::visitRange_expression(oRatioParser::Range_expressionContext* ctx) {
    arith_item * var;
    arith_item * min = visit(ctx->min).as<arith_item *>();
    arith_item * max = visit(ctx->max).as<arith_item *>();
    if (min->get_type()->_name.compare(REAL_KEYWORD) == 0 || max->get_type()->_name.compare(INT_KEYWORD) == 0) {
        var = _parser->_core->new_real();
    }
    else {
        var = _parser->_core->new_int();
    }
    bool assert_facts = _parser->_core->_net.add({
        _parser->_core->_net.geq(&var->get_arith_var(), &min->get_arith_var()),
        _parser->_core->_net.leq(&var->get_arith_var(), &max->get_arith_var())
    });
    assert(assert_facts && "invalid range expression..");
    return var;
}

antlrcpp::Any expression_visitor::visitConstructor_expression(oRatioParser::Constructor_expressionContext* ctx) {
    type * t = type_visitor(_parser).visit(ctx->type());
    std::vector<item*> exprs;
    std::vector<const type*> par_types;
    if (ctx->expr_list()) {
        for (const auto& expr : ctx->expr_list()->expr()) {
            item * i = expression_visitor(_parser, _env).visit(expr).as<item*>();
            exprs.push_back(i);
            par_types.push_back(i->get_type());
        }
    }

    constructor* c = t->get_constructor(par_types);
    if (!c) {
        _parser->_parser->notifyErrorListeners(ctx->start, "cannot find constructor..", nullptr);
        return nullptr;
    }
    return c->new_instance(_env, exprs);
}

antlrcpp::Any expression_visitor::visitEq_expression(oRatioParser::Eq_expressionContext* ctx) {
    item* i0 = visit(ctx->expr(0));
    item* i1 = visit(ctx->expr(1));
    return _parser->_core->eq(i0, i1);
}

antlrcpp::Any expression_visitor::visitLt_expression(oRatioParser::Lt_expressionContext* ctx) {
    arith_item* i0 = dynamic_cast<arith_item*> (visit(ctx->expr(0)).as<item*>());
    arith_item* i1 = dynamic_cast<arith_item*> (visit(ctx->expr(1)).as<item*>());
    return _parser->_core->lt(i0, i1);
}

antlrcpp::Any expression_visitor::visitLeq_expression(oRatioParser::Leq_expressionContext* ctx) {
    arith_item* i0 = dynamic_cast<arith_item*> (visit(ctx->expr(0)).as<item*>());
    arith_item* i1 = dynamic_cast<arith_item*> (visit(ctx->expr(1)).as<item*>());
    return _parser->_core->leq(i0, i1);
}

antlrcpp::Any expression_visitor::visitGeq_expression(oRatioParser::Geq_expressionContext* ctx) {
    arith_item* i0 = dynamic_cast<arith_item*> (visit(ctx->expr(0)).as<item*>());
    arith_item* i1 = dynamic_cast<arith_item*> (visit(ctx->expr(1)).as<item*>());
    return _parser->_core->geq(i0, i1);
}

antlrcpp::Any expression_visitor::visitGt_expression(oRatioParser::Gt_expressionContext* ctx) {
    arith_item* i0 = dynamic_cast<arith_item*> (visit(ctx->expr(0)).as<item*>());
    arith_item* i1 = dynamic_cast<arith_item*> (visit(ctx->expr(1)).as<item*>());
    return _parser->_core->gt(i0, i1);
}

antlrcpp::Any expression_visitor::visitNeq_expression(oRatioParser::Neq_expressionContext* ctx) {
    item* i0 = visit(ctx->expr(0));
    item* i1 = visit(ctx->expr(1));
    bool_item * eq = _parser->_core->eq(i0, i1);
    return _parser->_core->negate(eq);
}

antlrcpp::Any expression_visitor::visitImplication_expression(oRatioParser::Implication_expressionContext* ctx) {
    bool_item* i0 = dynamic_cast<bool_item*> (visit(ctx->expr(0)).as<item*>());
    bool_item* i1 = dynamic_cast<bool_item*> (visit(ctx->expr(1)).as<item*>());
    return _parser->_core->disjunction({ _parser->_core->negate(i0), i1 });
}

antlrcpp::Any expression_visitor::visitDisjunction_expression(oRatioParser::Disjunction_expressionContext* ctx) {
    std::vector<bool_item*> is;
    for (const auto& e : ctx->expr()) {
        is.push_back(dynamic_cast<bool_item*> (visit(e).as<item*>()));
    }
    return _parser->_core->disjunction(is);
}

antlrcpp::Any expression_visitor::visitConjunction_expression(oRatioParser::Conjunction_expressionContext* ctx) {
    std::vector<bool_item*> is;
    for (const auto& e : ctx->expr()) {
        is.push_back(dynamic_cast<bool_item*> (visit(e).as<item*>()));
    }
    return _parser->_core->conjunction(is);
}

antlrcpp::Any expression_visitor::visitExtc_one_expression(oRatioParser::Extc_one_expressionContext* ctx) {
    std::vector<bool_item*> is;
    for (const auto& e : ctx->expr()) {
        is.push_back(dynamic_cast<bool_item*> (visit(e).as<item*>()));
    }
    return _parser->_core->exactly_one(is);
}
