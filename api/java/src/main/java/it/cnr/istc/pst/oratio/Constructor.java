package it.cnr.istc.pst.oratio;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

public class Constructor implements Scope {

    final Solver solver;
    final Scope scope;
    final Map<String, Field> fields = new LinkedHashMap<>();
    final Field[] pars;

    Constructor(final Solver solver, final Scope scope, final Field... parameters) {
        this.solver = solver;
        this.scope = scope;
        this.pars = parameters;

        for (final Field par : parameters)
            fields.put(par.getName(), par);
    }

    @Override
    public Solver getSolver() {
        return solver;
    }

    @Override
    public Scope getScope() {
        return scope;
    }

    /**
     * Returns the field of this scope having the given name. Checks the enclosing
     * scope if the field is not found within this scope.
     *
     * @param name a string representing the name of the field.
     * @return a field having the given name.
     */
    @Override
    public Field getField(final String name) throws NoSuchFieldException {
        final Field field = fields.get(name);
        if (field != null) {
            return field;
        } else {
            return scope.getField(name);
        }
    }

    /**
     * Returns all the field defined within this scope.
     *
     * @return a map containing all the field defined within this scope indexed by
     *         the field's name.
     */
    @Override
    public Map<String, Field> getFields() {
        return Collections.unmodifiableMap(fields);
    }

    public Field[] getParemeters() {
        return pars;
    }
}
