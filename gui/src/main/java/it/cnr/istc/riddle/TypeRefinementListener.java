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

import java.util.ArrayList;
import java.util.List;
import org.antlr.v4.runtime.tree.TerminalNode;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
class TypeRefinementListener extends riddleBaseListener {

    private final Core core;
    private final riddleParser parser;
    private Scope scope = null;

    TypeRefinementListener(final Core core, final riddleParser parser) {
        this.core = core;
        this.parser = parser;
        this.scope = core;
    }

    @Override
    public void enterEnum_declaration(riddleParser.Enum_declarationContext ctx) {
        EnumType c_type = (EnumType) core.scopes.get(ctx);
        ctx.enum_constants().stream().filter(enum_constant -> (enum_constant.type() != null)).forEach(enum_constant -> c_type.enums.add((EnumType) new TypeVisitor(core, parser).visit(enum_constant.type())));
    }

    @Override
    public void enterClass_declaration(riddleParser.Class_declarationContext ctx) {
        // we set the type superclasses..
        scope = core.scopes.get(ctx);
        if (ctx.type_list() != null) {
            for (riddleParser.TypeContext type : ctx.type_list().type()) {
                ((Type) scope).superclasses.add(new TypeVisitor(core, parser).visit(type));
            }
        }
    }

    @Override
    public void exitClass_declaration(riddleParser.Class_declarationContext ctx) {
        // if the current type has no constructor..
        if (((Type) scope).constructors.isEmpty()) {
            // .. we define a default empty constructor..
            ((Type) scope).constructors.add(new Constructor(core, scope));
        }
        scope = scope.getScope();
    }

    @Override
    public void enterField_declaration(riddleParser.Field_declarationContext ctx) {
        // we add a field to the current scope..
        Type c_type = new TypeVisitor(core, parser).visit(ctx.type());
        ctx.variable_dec().forEach(dec -> defineField(new Field(c_type, dec.name.getText())));
    }

    @Override
    public void enterConstructor_declaration(riddleParser.Constructor_declarationContext ctx) {
        // we add a new constructor to the current type..
        // these are the parameters of the new constructor..
        Field[] parameters;
        if (ctx.typed_list() != null) {
            List<riddleParser.TypeContext> typed_list = ctx.typed_list().type();
            List<TerminalNode> ids = ctx.typed_list().ID();
            parameters = new Field[typed_list.size()];
            for (int i = 0; i < typed_list.size(); i++) {
                parameters[i] = new Field(new TypeVisitor(core, parser).visit(typed_list.get(i)), ids.get(i).getText());
            }
        } else {
            parameters = new Field[0];
        }

        Constructor constructor = new Constructor(core, scope, parameters);

        ((Type) scope).constructors.add(constructor);
        core.scopes.put(ctx, constructor);
        scope = constructor;
    }

    @Override
    public void exitConstructor_declaration(riddleParser.Constructor_declarationContext ctx) {
        // we restore the scope as the enclosing scope of the current scope..
        scope = scope.getScope();
    }

    @Override
    public void enterVoid_method_declaration(riddleParser.Void_method_declarationContext ctx) {
        // we add a new method without return type to the current type..
        // these are the parameters of the new method..
        Field[] parameters;
        if (ctx.typed_list() != null) {
            List<riddleParser.TypeContext> typed_list = ctx.typed_list().type();
            List<TerminalNode> ids = ctx.typed_list().ID();
            parameters = new Field[typed_list.size()];
            for (int i = 0; i < typed_list.size(); i++) {
                parameters[i] = new Field(new TypeVisitor(core, parser).visit(typed_list.get(i)), ids.get(i).getText());
            }
        } else {
            parameters = new Field[0];
        }
        Method method = new Method(core, scope, ctx.name.getText(), null, parameters);
        defineMethod(method);
        core.scopes.put(ctx, method);
        scope = method;
    }

    @Override
    public void exitVoid_method_declaration(riddleParser.Void_method_declarationContext ctx) {
        // we restore the scope as the enclosing scope of the current scope..
        scope = scope.getScope();
    }

    @Override
    public void enterType_method_declaration(riddleParser.Type_method_declarationContext ctx) {
        // we add a new method with a return type to the current type..
        // these are the parameters of the new method..
        Field[] parameters;
        if (ctx.typed_list() != null) {
            List<riddleParser.TypeContext> typed_list = ctx.typed_list().type();
            List<TerminalNode> ids = ctx.typed_list().ID();
            parameters = new Field[typed_list.size()];
            for (int i = 0; i < typed_list.size(); i++) {
                parameters[i] = new Field(new TypeVisitor(core, parser).visit(typed_list.get(i)), ids.get(i).getText());
            }
        } else {
            parameters = new Field[0];
        }
        Method method = new Method(core, scope, ctx.name.getText(), new TypeVisitor(core, parser).visit(ctx.type()), parameters);
        defineMethod(method);
        core.scopes.put(ctx, method);
        scope = method;
    }

    @Override
    public void exitType_method_declaration(riddleParser.Type_method_declarationContext ctx) {
        // we restore the scope as the enclosing scope of the current scope..
        scope = scope.getScope();
    }

    @Override
    public void enterPredicate_declaration(riddleParser.Predicate_declarationContext ctx) {
        // we add a new predicate to the current scope..
        // these are the parameters of the new predicate..
        Field[] parameters;
        if (ctx.typed_list() != null) {
            List<riddleParser.TypeContext> typed_list = ctx.typed_list().type();
            List<TerminalNode> ids = ctx.typed_list().ID();
            parameters = new Field[typed_list.size()];
            for (int i = 0; i < typed_list.size(); i++) {
                parameters[i] = new Field(new TypeVisitor(core, parser).visit(typed_list.get(i)), ids.get(i).getText());
            }
        } else {
            parameters = new Field[0];
        }
        Predicate predicate = new Predicate(core, scope, ctx.name.getText(), parameters);
        if (ctx.predicate_list() != null) {
            ctx.predicate_list().qualified_predicate().forEach(type -> predicate.superclasses.add(new TypeVisitor(core, parser).visit(type)));
        }
        definePredicate(predicate);
        core.scopes.put(ctx, predicate);
        scope = predicate;
    }

    @Override
    public void exitPredicate_declaration(riddleParser.Predicate_declarationContext ctx) {
        // we restore the scope as the enclosing scope of the current scope..
        scope = scope.getScope();
    }

    private void defineField(Field field) {
        if (scope instanceof Core) {
            ((Core) scope).fields.put(field.name, field);
        } else if (scope instanceof Type) {
            ((Type) scope).fields.put(field.name, field);
        }
    }

    public void defineMethod(Method method) {
        if (scope instanceof Core) {
            ((Core) scope).methods.putIfAbsent(method.name, new ArrayList<>()).add(method);
        } else if (scope instanceof Type) {
            ((Type) scope).methods.putIfAbsent(method.name, new ArrayList<>()).add(method);
        }
    }

    public void definePredicate(Predicate predicate) {
        if (scope instanceof Core) {
            ((Core) scope).predicates.put(predicate.name, predicate);
        } else if (scope instanceof Type) {
            ((Type) scope).predicates.put(predicate.name, predicate);
        }
    }
}
