package it.cnr.istc.pst.oratio.riddle;

class TypeDeclarationListener extends riddleBaseListener {

    private final Core core;
    private Scope scope = null;

    TypeDeclarationListener(final Core core) {
        this.core = core;
        this.scope = core;
    }

    @Override
    public void enterTypedef_declaration(riddleParser.Typedef_declarationContext ctx) {
        defineType(new Typedef(core, scope, new TypeVisitor(core).visit(ctx.primitive_type()), ctx.name.getText()));
    }

    @Override
    public void enterEnum_declaration(riddleParser.Enum_declarationContext ctx) {
        // A new enum type has been defined..
        EnumType c_type = new EnumType(core, scope, ctx.name.getText());
        core.scopes.put(ctx, c_type);
        // We add the enum values..
        ctx.enum_constants().forEach(enum_constant -> enum_constant.StringLiteral()
                .forEach(tn -> c_type.vals.add(tn.getSymbol().getText())));
        defineType(c_type);
    }

    @Override
    public void enterClass_declaration(riddleParser.Class_declarationContext ctx) {
        // A new class has been defined..
        Type c_type = new Type(core, scope, ctx.name.getText());
        core.scopes.put(ctx, c_type);
        defineType(c_type);
        scope = c_type;
    }

    @Override
    public void exitClass_declaration(riddleParser.Class_declarationContext ctx) {
        scope = scope.getScope();
    }

    private void defineType(Type type) {
        if (scope instanceof Core)
            ((Core) scope).types.put(type.name, type);
        else if (scope instanceof Type)
            ((Type) scope).types.put(type.name, type);
    }

    @Override
    public void enterClass_type(riddleParser.Class_typeContext ctx) {
        core.scopes.put(ctx, scope);
    }
}
