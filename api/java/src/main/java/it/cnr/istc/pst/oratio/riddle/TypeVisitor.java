package it.cnr.istc.pst.oratio.riddle;

import org.antlr.v4.runtime.tree.TerminalNode;

class TypeVisitor extends riddleBaseVisitor<Type> {

    private final Core core;

    TypeVisitor(final Core core) {
        this.core = core;
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
