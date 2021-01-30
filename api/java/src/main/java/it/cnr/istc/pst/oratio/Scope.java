package it.cnr.istc.pst.oratio;

import java.util.Collection;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Stream;

public interface Scope {

    /**
     * Returns the solver enclosing this scope.
     *
     * @return the solver enclosing this scope.
     */
    public Solver getSolver();

    /**
     * Returns the immediately enclosing scope of the underlying scope. If the
     * underlying scope is a top level scope this method returns the solver.
     *
     * @return the immediately enclosing scope of the underlying scope.
     */
    public Scope getScope();

    /**
     * Returns a {@code Field} object that reflects the specified public member
     * field of this {@code Scope} object.The {@code name} parameter is a
     * {@code String} specifying the simple name of the desired field.
     *
     * @param name the field name.
     * @return the {@code Field} object of this class specified by {@code name}.
     * @throws java.lang.NoSuchFieldException if the given field cannot be found.
     */
    public Field getField(final String name) throws NoSuchFieldException;

    /**
     * Returns a {@code Map} containing all the fields defined in the current scope.
     * The map has the field name as key.
     *
     * @return a {@code Map} containing all the fields defined in the current scope.
     */
    public Map<String, Field> getFields();

    /**
     * Returns a {@code Method} object that reflects the specified public member
     * method of the class or interface represented by this {@code Scope} object.The
     * {@code name} parameter is a {@code String} specifying the simple name of the
     * desired method. The {@code parameterTypes} parameter is an array of
     * {@code Type} objects that identify the method's formal parameter types, in
     * declared order. If {@code parameterTypes} is {@code null}, it is treated as
     * if it were an empty array.
     *
     * @param name            the name of the method.
     * @param parameter_types the list of parameters.
     * @return the {@code Method} object that matches the specified {@code name} and
     *         {@code parameterTypes}.
     * @throws java.lang.NoSuchMethodException if the given method cannot be found.
     */
    public default Method getMethod(final String name, final Type... parameter_types) throws NoSuchMethodException {
        if (Stream.of(parameter_types).anyMatch(Objects::isNull))
            throw new IllegalArgumentException("parameter types cannot be null");
        return getScope().getMethod(name, parameter_types);
    }

    /**
     * Returns a collection containing {@code Method} objects reflecting all the
     * public methods of the class or interface represented by this {@code
     * Scope} object.
     *
     * @return the collection of {@code Method} objects representing the public
     *         methods of this object.
     */
    public default Collection<Method> getMethods() {
        return getScope().getMethods();
    }

    /**
     * Returns a predicate whose scope is within this {@code Type} object scope.If a
     * predicate is not found within this scope, the enclosing scope is checked.
     *
     * @param name the name of the predicate to find.
     * @return a predicate having the given name.
     * @throws java.lang.ClassNotFoundException if the given predicate cannot be
     *                                          found.
     */
    public default Predicate getPredicate(final String name) throws ClassNotFoundException {
        return getScope().getPredicate(name);
    }

    /**
     * Returns all the predicates enclosed by this {@code Type} object mapped by
     * their names.
     *
     * @return a map containing all the predicates enclosed by this type having type
     *         names as indexes.
     */
    public default Map<String, Predicate> getPredicates() {
        return getScope().getPredicates();
    }

    /**
     * Returns a type whose scope is within this {@code Scope} object scope.If a
     * type is not found within this scope, the enclosing scope is checked.
     *
     * @param name the name of the type to find.
     * @return a type having the given name.
     * @throws java.lang.ClassNotFoundException if the given type cannot be found.
     */
    public default Type getType(final String name) throws ClassNotFoundException {
        return getScope().getType(name);
    }

    /**
     * Returns all the types enclosed by this {@code Scope} object mapped by their
     * names.
     *
     * @return a map containing all the types enclosed by this type having type
     *         names as indexes.
     */
    public default Map<String, Type> getTypes() {
        return getScope().getTypes();
    }
}
