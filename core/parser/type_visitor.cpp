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

#include "type_visitor.h"
#include "parser.h"
#include "../core.h"
#include "../type.h"
#include "../field.h"

using namespace oratio;

type_visitor::type_visitor(parser * const p) : _parser(p) { }

type_visitor::~type_visitor() { }

antlrcpp::Any type_visitor::visitLiteral_expression(oRatioParser::Literal_expressionContext* ctx) {
    if (ctx->literal()->numeric) {
        if (ctx->literal()->numeric->getText().find('.') != ctx->literal()->numeric->getText().npos) {
            return _parser->_core->get_type(REAL_KEYWORD);
        }
        else {
            return _parser->_core->get_type(INT_KEYWORD);
        }
    }
    else if (ctx->literal()->string) {
        return _parser->_core->get_type(STRING_KEYWORD);
    }
    else if (ctx->literal()->t) {
        return _parser->_core->get_type(BOOL_KEYWORD);
    }
    else if (ctx->literal()->f) {
        return _parser->_core->get_type(BOOL_KEYWORD);
    }
    else {
        std::cerr << "the primitive type has not been found.." << std::endl;
        return nullptr;
    }
}

antlrcpp::Any type_visitor::visitCast_expression(oRatioParser::Cast_expressionContext* ctx) {
    return visit(ctx->type());
}

antlrcpp::Any type_visitor::visitPrimitive_type(oRatioParser::Primitive_typeContext* ctx) {
    return _parser->_core->get_type(ctx->getText());
}

antlrcpp::Any type_visitor::visitClass_type(oRatioParser::Class_typeContext* ctx) {
    scope * s = _parser->_scopes.at(ctx);
    for (const auto& id : ctx->ID()) {
        s = s->get_type(id->getText());
        if (!s) {
            _parser->_parser->notifyErrorListeners(id->getSymbol(), "cannot find symbol..", nullptr);
            return nullptr;
        }
    }
    return static_cast<type*> (s);
}

antlrcpp::Any type_visitor::visitQualified_id(oRatioParser::Qualified_idContext* ctx) {
    const scope * s = _parser->_scopes.at(ctx);
    if (ctx->t) {
        s = s->get_field(THIS_KEYWORD)->_type;
    }
    for (const auto& id : ctx->ID()) {
        s = s->get_field(id->getText())->_type;
        if (!s) {
            _parser->_parser->notifyErrorListeners(id->getSymbol(), "cannot find symbol..", nullptr);
            return nullptr;
        }
    }
    return const_cast<type*> (static_cast<const type*> (s));
}

antlrcpp::Any type_visitor::visitQualified_id_expression(oRatioParser::Qualified_id_expressionContext* ctx) {
    const scope * s = _parser->_scopes.at(ctx);
    if (ctx->qualified_id()->t) {
        s = s->get_field(THIS_KEYWORD)->_type;
    }
    for (const auto& id : ctx->qualified_id()->ID()) {
        s = s->get_field(id->getText())->_type;
        if (!s) {
            _parser->_parser->notifyErrorListeners(id->getSymbol(), "cannot find symbol..", nullptr);
            return nullptr;
        }
    }
    return const_cast<type*> (static_cast<const type*> (s));
}

antlrcpp::Any type_visitor::visitConstructor_expression(oRatioParser::Constructor_expressionContext* ctx) {
    return visit(ctx->type());
}
