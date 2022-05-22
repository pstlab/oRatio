package it.cnr.istc.pst.oratio;

public class Field {

    final Type type;
    final String name;

    Field(final Type type, final String name) {
        this.type = type;
        this.name = name;
    }

    public Type getType() {
        return type;
    }

    public String getName() {
        return name;
    }
}
