package it.cnr.istc.pst.oratio;

public class Predicate extends Type {

    final Field[] pars;

    Predicate(final Solver solver, final Scope scope, final String name, final Field... parameters) {
        super(solver, scope, name, false);
        this.pars = parameters;

        for (final Field par : parameters)
            fields.put(par.name, par);
    }

    public Field[] getParemeters() {
        return pars;
    }
}
