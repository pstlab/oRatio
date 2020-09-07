package it.cnr.istc.pst.oratio.riddle;

public class Predicate extends Type {

    final Field[] pars;

    Predicate(final Core core, final Scope scope, final String name, final Field... parameters) {
        super(core, scope, name);
        this.pars = parameters;

        for (Field par : parameters)
            fields.put(par.name, par);
    }

    public Field[] getParemeters() {
        return pars;
    }
}
