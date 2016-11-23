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

#include "type_declaration_listener.h"
#include "parser.h"
#include "../core.h"
#include "type_visitor.h"
#include "typedef_type.h"
#include "enum_type.h"

using namespace oratio;

type_declaration_listener::type_declaration_listener(parser * const p) : _parser(p), _scope(p->_core) { }

type_declaration_listener::~type_declaration_listener() { }

void type_declaration_listener::enterCompilation_unit(oRatioParser::Compilation_unitContext* ctx) {
    _parser->_scopes.insert({ ctx, _parser->_core });
}

void type_declaration_listener::enterTypedef_declaration(oRatioParser::Typedef_declarationContext* ctx) {
    // A new typedef type has been declared..
    typedef_type* td = new typedef_type(_parser->_core, _scope, ctx->name->getText(), type_visitor(_parser).visit(ctx->primitive_type()), ctx->expr());
    if (core * c = dynamic_cast<core*> (_scope)) {
        c->_types.insert({ ctx->name->getText(), td });
    }
    else if (type * t = dynamic_cast<type*> (_scope)) {
        t->_types.insert({ ctx->name->getText(), td });
    }
}

void type_declaration_listener::enterEnum_declaration(oRatioParser::Enum_declarationContext* ctx) {
    // A new enum type has been declared..
    enum_type* et = new enum_type(_parser->_core, _scope, ctx->name->getText());
    _parser->_scopes.insert({ ctx, et });

    // We add the enum values..
    for (const auto& c : ctx->enum_constants()) {
        for (const auto& l : c->StringLiteral()) {
            std::string c_enum = l->getText();
            et->add_enum(_parser->_core->new_string(c_enum));
        }
    }
    if (core * c = dynamic_cast<core*> (_scope)) {
        c->_types.insert({ ctx->name->getText(), et });
    }
    else if (type * t = dynamic_cast<type*> (_scope)) {
        t->_types.insert({ ctx->name->getText(), et });
    }
}

void type_declaration_listener::enterClass_declaration(oRatioParser::Class_declarationContext* ctx) {
    // A new type has been declared..
    type* c_t = new type(_parser->_core, _scope, ctx->name->getText());
    _parser->_scopes.insert({ ctx, c_t });
    if (core * e = dynamic_cast<core*> (_scope)) {
        e->_types.insert({ ctx->name->getText(), c_t });
    }
    else if (type * t = dynamic_cast<type*> (_scope)) {
        t->_types.insert({ ctx->name->getText(), c_t });
    }

    _scope = c_t;
}

void type_declaration_listener::exitClass_declaration(oRatioParser::Class_declarationContext* ctx) {
    _scope = _scope->_scope;
}

void type_declaration_listener::enterClass_type(oRatioParser::Class_typeContext* ctx) {
    _parser->_scopes.insert({ ctx, _scope });
}
