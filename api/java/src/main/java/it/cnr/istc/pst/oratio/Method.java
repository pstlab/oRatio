package it.cnr.istc.pst.oratio;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

public class Method implements Scope {

    final Solver solver;
    final Scope scope;
    final Map<String, Field> fields = new LinkedHashMap<>();
    final String name;
    final Type return_type;
    final Field[] pars;

    Method(final Solver solver, final Scope scope, final String name, final Type return_type,
            final Field... parameters) {
        this.solver = solver;
        this.scope = scope;
        this.name = name;
        this.return_type = return_type;
        this.pars = parameters;

        for (final Field par : parameters)
            fields.put(par.name, par);
    }

    @Override
    public Solver getSolver() {
        return solver;
    }

    @Override
    public Scope getScope() {
        return scope;
    }

    @Override
    public Field getField(final String name) throws NoSuchFieldException {
        final Field field = fields.get(name);
        if (field != null) {
            return field;
        } else {
            return scope.getField(name);
        }
    }

    @Override
    public Map<String, Field> getFields() {
        return Collections.unmodifiableMap(fields);
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
