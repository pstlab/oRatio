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

#include "statement_visitor.h"
#include "../env.h"
#include "../scope.h"
#include "../item.h"
#include "parser.h"
#include "expression_visitor.h"
#include "type_visitor.h"
#include "../type.h"
#include "../core.h"
#include "../predicate.h"
#include "../atom.h"
#include "../field.h"
#include "../disjunction.h"

using namespace oratio;

statement_visitor::statement_visitor(parser * const p, env * const e) : _parser(p), _env(e) { }

statement_visitor::~statement_visitor() { }

antlrcpp::Any statement_visitor::visitCompilation_unit(oRatioParser::Compilation_unitContext* ctx) {
    for (const auto& s : ctx->statement()) {
        if (!visit(s).as<bool>()) {
            return false;
        }
    }
    return true;
}

antlrcpp::Any statement_visitor::visitBlock(oRatioParser::BlockContext* ctx) {
    for (const auto& s : ctx->statement()) {
        if (!visit(s).as<bool>()) {
            return false;
        }
    }
    return true;
}

antlrcpp::Any statement_visitor::visitConjunction(oRatioParser::ConjunctionContext* ctx) {
    return visit(ctx->block()).as<bool>();
}

antlrcpp::Any statement_visitor::visitAssignment_statement(oRatioParser::Assignment_statementContext* ctx) {
    env * e = _env;
    if (ctx->object) {
        if (ctx->object->t) {
            e = e->get(THIS_KEYWORD);
        }
        for (const auto& id : ctx->object->ID()) {
            e = e->get(id->getText());
            if (!e) {
                _parser->_parser->notifyErrorListeners(id->getSymbol(), "cannot find symbol..", nullptr);
                return nullptr;
            }
        }
    }

    e->_items.insert({ ctx->field->getText(), expression_visitor(_parser, _env).visit(ctx->expr()) });
    return true;
}

antlrcpp::Any statement_visitor::visitLocal_variable_statement(oRatioParser::Local_variable_statementContext* ctx) {
    type * t = type_visitor(_parser).visit(ctx->type());
    for (const auto& dec : ctx->variable_dec()) {
        if (dec->expr()) {
            _env->_items.insert({ dec->name->getText(), expression_visitor(_parser, _env).visit(dec->expr()) });
        }
        else if (t->_primitive) {
            _env->_items.insert({ dec->name->getText(), t->new_instance(_env) });
        }
        else {
            _env->_items.insert({ dec->name->getText(), t->new_existential() });
        }
    }
    return true;
}

antlrcpp::Any statement_visitor::visitExpression_statement(oRatioParser::Expression_statementContext* ctx) {
    return _parser->_core->assert_facts(std::vector<bool_item*>({ expression_visitor(_parser, _env).visit(ctx->expr()) }));
}

antlrcpp::Any statement_visitor::visitFormula_statement(oRatioParser::Formula_statementContext* ctx) {
    env * e = _env;
    predicate * p;
    std::unordered_map<std::string, item*> assignments;
    if (ctx->object) {
        if (ctx->object->t) {
            e = e->get(THIS_KEYWORD);
        }
        for (const auto& id : ctx->object->ID()) {
            e = e->get(id->getText());
            if (!e) {
                _parser->_parser->notifyErrorListeners(id->getSymbol(), "cannot find symbol..", nullptr);
                return nullptr;
            }
        }
        p = dynamic_cast<item*> (e)->get_type()->get_predicate(ctx->predicate->getText());
        item* i = dynamic_cast<item*> (e);
        if (dynamic_cast<enum_item*> (i)) {
            assignments.insert({ "scope", i });
        }
        else {
            assignments.insert({ "scope", _parser->_core->new_enum(i->get_type(), std::unordered_set<item*>(
                {i})) });
        }
    }
    else {
        p = _parser->_scopes.at(ctx)->_scope->get_predicate(ctx->predicate->getText());
        if (p->_scope != _parser->_core) {
            // we inherit the scope..
            assignments.insert({ "scope", e->get("scope") });
        }
    }

    if (ctx->assignment_list()) {
        for (const auto& a : ctx->assignment_list()->assignment()) {
            assignments.insert({ a->field->getText(), expression_visitor(_parser, _env).visit(a->expr()) });
        }
    }

    atom* a = static_cast<atom*> (p->new_instance(e));
    a->_items.insert(assignments.begin(), assignments.end());

    std::set<const predicate*> ps;
    std::queue<predicate*> q;
    q.push(p);
    while (!q.empty()) {
        ps.insert(q.front());
        for (const auto& sp : q.front()->get_supertypes()) {
            q.push(static_cast<predicate*> (sp));
        }
        q.pop();
    }

    for (const auto& c_p : ps) {
        for (const auto& arg : c_p->get_args()) {
            if (a->_items.find(arg->_name) == a->_items.end()) {
                if (arg->_type->_primitive) {
                    a->_items.insert({ arg->_name, const_cast<type*> (arg->_type)->new_instance(_env) });
                }
                else {
                    a->_items.insert({ arg->_name, const_cast<type*> (arg->_type)->new_existential() });
                }
            }
        }
    }

    if (ctx->fact) {
        if (!_parser->_core->new_fact(a)) {
            return false;
        }
    }
    else if (ctx->goal) {
        if (!_parser->_core->new_goal(a)) {
            return false;
        }
    }

    _env->_items.insert({ ctx->name->getText(), a });
    return true;
}

antlrcpp::Any statement_visitor::visitReturn_statement(oRatioParser::Return_statementContext* ctx) {
    _env->_items.insert({ RETURN_KEYWORD, expression_visitor(_parser, _env).visit(ctx->expr()) });
    return true;
}

antlrcpp::Any statement_visitor::visitDisjunction_statement(oRatioParser::Disjunction_statementContext* ctx) {
    return _parser->_core->new_disjunction(_env, static_cast<oratio::disjunction*> (_parser->_scopes.at(ctx)));
}
