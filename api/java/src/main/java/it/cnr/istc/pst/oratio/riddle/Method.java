package it.cnr.istc.pst.oratio.riddle;

public class Method extends BaseScope {

    final String name;
    final Type return_type;
    final Field[] pars;

    Method(final Core core, final Scope scope, final String name, final Type return_type, final Field... parameters) {
        super(core, scope);
        this.name = name;
        this.return_type = return_type;
        this.pars = parameters;

        for (Field par : parameters)
            fields.put(par.name, par);
    }

    public String getName() {
        return name;
    }

    public Type getReturnType() {
        return return_type;
    }

    public Field[] getParemeters() {
        return pars;
    }
}
