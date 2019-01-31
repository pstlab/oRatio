/*
 * Copyright (C) 2018 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
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
package it.cnr.istc.riddle;

import org.antlr.v4.runtime.tree.TerminalNode;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
class TypeVisitor extends riddleBaseVisitor<Type> {

    private final Core core;
    private final riddleParser parser;

    TypeVisitor(final Core core, final riddleParser parser) {
        this.core = core;
        this.parser = parser;
    }

    @Override
    public Type visitPrimitive_type(final riddleParser.Primitive_typeContext ctx) {
        try {
            return core.getType(ctx.getText());
        } catch (ClassNotFoundException ex) {
        }
        return null;
    }

    @Override
    public Type visitClass_type(final riddleParser.Class_typeContext ctx) {
        Scope c_scope = core.scopes.get(ctx);
        for (TerminalNode id : ctx.ID())
            try {
                c_scope = c_scope.getType(id.getText());
            } catch (ClassNotFoundException ex) {
            }
        return (Type) c_scope;
    }

    @Override
    public Type visitQualified_id(final riddleParser.Qualified_idContext ctx) {
        Scope c_scope = core.scopes.get(ctx);
        for (TerminalNode id : ctx.ID())
            try {
                c_scope = c_scope.getField(id.getText()).type;
            } catch (NoSuchFieldException ex) {
                return null;
            }
        return (Type) c_scope;
    }

    @Override
    public Type visitQualified_predicate(riddleParser.Qualified_predicateContext ctx) {
        Scope c_scope = core.scopes.get(ctx);
        if (ctx.class_type() != null)
            for (TerminalNode id : ctx.class_type().ID())
                try {
                    c_scope = c_scope.getType(id.getText());
                } catch (ClassNotFoundException ex) {
                    return null;
                }
        try {
            return c_scope.getPredicate(ctx.ID().getText());
        } catch (ClassNotFoundException ex) {
            return null;
        }
    }
}
