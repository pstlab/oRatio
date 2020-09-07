package it.cnr.istc.pst.oratio.riddle;

public class Typedef extends Type {

    final Type primitive_type;

    Typedef(final Core core, final Scope scope, final Type primitive_type, final String name) {
        super(core, scope, name);
        this.primitive_type = primitive_type;
    }

    public Type getPrimitiveType() {
        return primitive_type;
    }
}
