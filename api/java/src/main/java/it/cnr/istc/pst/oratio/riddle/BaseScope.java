package it.cnr.istc.pst.oratio.riddle;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

public class BaseScope implements Scope {

    final Core core;
    final Scope scope;
    final Map<String, Field> fields = new LinkedHashMap<>();

    BaseScope(final Core core, final Scope scope) {
        if (core == null)
            throw new IllegalArgumentException("core cannot be null");
        if (scope == null)
            throw new IllegalArgumentException("scope cannot be null");
        this.core = core;
        this.scope = scope;
    }

    @Override
    public Core getCore() {
        return core;
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
        Field field = fields.get(name);
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
}
