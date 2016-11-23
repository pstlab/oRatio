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

#include "type_refinement_listener.h"
#include "parser.h"
#include "enum_type.h"
#include "type_visitor.h"
#include "default_constructor.h"
#include "instantiated_field.h"
#include "defined_constructor.h"
#include "defined_method.h"
#include "defined_predicate.h"
#include "defined_conjunction.h"
#include "../core.h"
#include "expression_visitor.h"

using namespace oratio;

type_refinement_listener::type_refinement_listener(parser * const p) : _parser(p) { }

type_refinement_listener::~type_refinement_listener() { }

void type_refinement_listener::enterCompilation_unit(oRatioParser::Compilation_unitContext* ctx) {
    _scope = _parser->_scopes.at(ctx);
}

void type_refinement_listener::enterEnum_declaration(oRatioParser::Enum_declarationContext* ctx) {
    enum_type* et = static_cast<enum_type*> (_parser->_scopes.at(ctx));
    for (const auto& ec : ctx->enum_constants()) {
        if (ec->type()) {
            et->_enums.push_back(type_visitor(_parser).visit(ec->type()).as<enum_type*>());
        }
    }
}

void type_refinement_listener::enterClass_declaration(oRatioParser::Class_declarationContext* ctx) {
    // we set the type superclasses..
    _scope = _parser->_scopes.at(ctx);
    if (ctx->type_list()) {
        type* t = static_cast<type*> (_scope);
        for (const auto& st : ctx->type_list()->type()) {
            t->_supertypes.push_back(type_visitor(_parser).visit(st).as<type*>());
        }
    }
}

void type_refinement_listener::exitClass_declaration(oRatioParser::Class_declarationContext* ctx) {
    // if the current type has no constructor..
    type* t = static_cast<type*> (_scope);
    if (t->_constructors.empty()) {
        // .. we define a default empty constructor..
        t->_constructors.push_back(new default_constructor(_parser->_core, _scope));
    }
    _scope = _scope->_scope;
}

void type_refinement_listener::enterField_declaration(oRatioParser::Field_declarationContext* ctx) {
    // we add a field to the current scope..
    type * t = type_visitor(_parser).visit(ctx->type());
    for (const auto& dec : ctx->variable_dec()) {
        if (dec->expr()) {
            instantiated_field* inst_f = new instantiated_field(t, dec->name->getText(), dec->expr());
            _scope->_fields.insert({ dec->name->getText(), inst_f });
        }
        else {
            field* f = new field(t, dec->name->getText());
            _scope->_fields.insert({ dec->name->getText(), f });
        }
    }
}

void type_refinement_listener::enterConstructor_declaration(oRatioParser::Constructor_declarationContext* ctx) {
    // we add a new constructor to the current type..
    // these are the parameters of the new constructor..
    std::vector<field*> args;
    if (ctx->typed_list()) {
        std::vector<oRatioParser::TypeContext*> types = ctx->typed_list()->type();
        std::vector<antlr4::tree::TerminalNode*> ids = ctx->typed_list()->ID();
        for (unsigned int i = 0; i < types.size(); i++) {
            args.push_back(new field(type_visitor(_parser).visit(types[i]).as<type*>(), ids[i]->getText()));
        }
    }

    defined_constructor* c = new defined_constructor(_parser->_core, _scope, args, ctx->initializer_element(), ctx->block());
    static_cast<type*> (_scope)->_constructors.push_back(c);
    _parser->_scopes.insert({ ctx, c });
    _scope = c;
}

void type_refinement_listener::exitConstructor_declaration(oRatioParser::Constructor_declarationContext* ctx) {
    // we restore the scope as the enclosing scope of the current scope..
    _scope = _scope->_scope;
}

void type_refinement_listener::enterVoid_method_declaration(oRatioParser::Void_method_declarationContext* ctx) {
    // we add a new method without return type to the current scope..
    // these are the parameters of the new method..
    std::vector<field*> args;
    if (ctx->typed_list()) {
        std::vector<oRatioParser::TypeContext*> types = ctx->typed_list()->type();
        std::vector<antlr4::tree::TerminalNode*> ids = ctx->typed_list()->ID();
        for (unsigned int i = 0; i < types.size(); i++) {
            args.push_back(new field(type_visitor(_parser).visit(types[i]).as<type*>(), ids[i]->getText()));
        }
    }

    defined_method* m = new defined_method(_parser->_core, _scope, ctx->name->getText(), args, ctx->block());
    if (core * c = dynamic_cast<core*> (_scope)) {
        if (c->_methods.find(ctx->name->getText()) == c->_methods.end()) {
            c->_methods.insert({ ctx->name->getText(), *new std::vector<method*>() });
        }
        c->_methods.at(ctx->name->getText()).push_back(m);
    }
    else if (type * t = dynamic_cast<type*> (_scope)) {
        if (t->_methods.find(ctx->name->getText()) == t->_methods.end()) {
            t->_methods.insert({ ctx->name->getText(), *new std::vector<method*>() });
        }
        t->_methods.at(ctx->name->getText()).push_back(m);
    }
    _parser->_scopes.insert({ ctx, m });
    _scope = m;
}

void type_refinement_listener::exitVoid_method_declaration(oRatioParser::Void_method_declarationContext* ctx) {
    // we restore the scope as the enclosing scope of the current scope..
    _scope = _scope->_scope;
}

void type_refinement_listener::enterType_method_declaration(oRatioParser::Type_method_declarationContext* ctx) {
    // we add a new method with a return type to the current scope..
    // these are the parameters of the new method..
    type* return_type = type_visitor(_parser).visit(ctx->type());
    std::vector<field*> args;
    if (ctx->typed_list()) {
        std::vector<oRatioParser::TypeContext*> types = ctx->typed_list()->type();
        std::vector<antlr4::tree::TerminalNode*> ids = ctx->typed_list()->ID();
        for (unsigned int i = 0; i < types.size(); i++) {
            args.push_back(new field(type_visitor(_parser).visit(types[i]).as<type*>(), ids[i]->getText()));
        }
    }

    defined_method* m = new defined_method(_parser->_core, _scope, ctx->name->getText(), args, ctx->block(), return_type);
    if (core * c = dynamic_cast<core*> (_scope)) {
        if (c->_methods.find(ctx->name->getText()) == c->_methods.end()) {
            c->_methods.insert({ ctx->name->getText(), *new std::vector<method*>() });
        }
        c->_methods.at(ctx->name->getText()).push_back(m);
    }
    else if (type * t = dynamic_cast<type*> (_scope)) {
        if (t->_methods.find(ctx->name->getText()) == t->_methods.end()) {
            t->_methods.insert({ ctx->name->getText(), *new std::vector<method*>() });
        }
        t->_methods.at(ctx->name->getText()).push_back(m);
    }
    _parser->_scopes.insert({ ctx, m });
    _scope = m;
}

void type_refinement_listener::exitType_method_declaration(oRatioParser::Type_method_declarationContext* ctx) {
    // we restore the scope as the enclosing scope of the current scope..
    _scope = _scope->_scope;
}

void type_refinement_listener::enterPredicate_declaration(oRatioParser::Predicate_declarationContext* ctx) {
    // we add a new predicate to the current scope..
    // these are the parameters of the new predicate..
    std::vector<field*> args;
    if (ctx->typed_list()) {
        std::vector<oRatioParser::TypeContext*> types = ctx->typed_list()->type();
        std::vector<antlr4::tree::TerminalNode*> ids = ctx->typed_list()->ID();
        for (unsigned int i = 0; i < types.size(); i++) {
            args.push_back(new field(type_visitor(_parser).visit(types[i]).as<type*>(), ids[i]->getText()));
        }
    }
    defined_predicate* p = new defined_predicate(_parser->_core, _scope, ctx->name->getText(), args, ctx->block());

    if (ctx->type_list()) {
        for (const auto& t : ctx->type_list()->type()) {
            p->_supertypes.push_back(type_visitor(_parser).visit(t));
        }
    }

    std::set<type*> ts;
    std::queue<type*> q;
    if (core * c = dynamic_cast<core*> (_scope)) {
        c->_predicates.insert({ ctx->name->getText(), p });
    }
    else if (type * t = dynamic_cast<type*> (_scope)) {
        t->_predicates.insert({ ctx->name->getText(), p });
        q.push(t);
    }
    while (!q.empty()) {
        ts.insert(q.front());
        for (const auto& st : q.front()->get_supertypes()) {
            q.push(st);
        }
        q.pop();
    }

    for (const auto& t : ts) {
        t->predicate_defined(p);
    }
    _parser->_scopes.insert({ ctx, p });
    _scope = p;
}

void type_refinement_listener::exitPredicate_declaration(oRatioParser::Predicate_declarationContext* ctx) {
    // we restore the scope as the enclosing scope of the current scope..
    _scope = _scope->_scope;
}

void type_refinement_listener::enterDisjunction_statement(oRatioParser::Disjunction_statementContext* ctx) {
    oratio::disjunction* d = new oratio::disjunction(_parser->_core, _scope);
    _parser->_scopes.insert({ ctx, d });
    _scope = d;
}

void type_refinement_listener::exitDisjunction_statement(oRatioParser::Disjunction_statementContext* ctx) {
    // we restore the scope as the enclosing scope of the current scope..
    _scope = _scope->_scope;
}

void type_refinement_listener::enterConjunction(oRatioParser::ConjunctionContext* ctx) {
    arith_item* cost;
    if (ctx->cost) {
        cost = dynamic_cast<arith_item*> (expression_visitor(_parser, _parser->_core).visit(ctx->cost).as<item*>());
    }
    else {
        cost = _parser->_core->new_real(1);
    }
    defined_conjunction* c = new defined_conjunction(_parser->_core, _scope, cost->get_arith_var(), ctx->block());
    _parser->_scopes.insert({ ctx, c });
    _scope = c;
}

void type_refinement_listener::exitConjunction(oRatioParser::ConjunctionContext* ctx) {
    // we restore the scope as the enclosing scope of the current scope..
    _scope = _scope->_scope;
}

void type_refinement_listener::enterLocal_variable_statement(oRatioParser::Local_variable_statementContext* ctx) {
    _parser->_scopes.insert({ ctx, _scope });
}

void type_refinement_listener::enterAssignment_statement(oRatioParser::Assignment_statementContext* ctx) {
    _parser->_scopes.insert({ ctx, _scope });
}

void type_refinement_listener::enterExpression_statement(oRatioParser::Expression_statementContext* ctx) {
    _parser->_scopes.insert({ ctx, _scope });
}

void type_refinement_listener::enterFormula_statement(oRatioParser::Formula_statementContext* ctx) {
    _parser->_scopes.insert({ ctx, _scope });
}

void type_refinement_listener::enterReturn_statement(oRatioParser::Return_statementContext* ctx) {
    _parser->_scopes.insert({ ctx, _scope });
}

void type_refinement_listener::enterQualified_id_expression(oRatioParser::Qualified_id_expressionContext* ctx) {
    _parser->_scopes.insert({ ctx, _scope });
}

void type_refinement_listener::enterFunction_expression(oRatioParser::Function_expressionContext* ctx) {
    _parser->_scopes.insert({ ctx, _scope });
}
