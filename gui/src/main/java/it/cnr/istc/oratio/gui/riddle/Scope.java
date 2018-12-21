/*
 * Copyright (C) 2018 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package it.cnr.istc.oratio.gui.riddle;

import java.util.Collection;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Stream;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public interface Scope {

    /**
     * Returns the core enclosing this scope.
     *
     * @return the core enclosing this scope.
     */
    public Core getCore();

    /**
     * Returns the immediately enclosing scope of the underlying scope. If the
     * underlying scope is a top level scope this method returns the solver.
     *
     * @return the immediately enclosing scope of the underlying scope.
     */
    public Scope getScope();

    /**
     * Returns an {@code IField} object that reflects the specified public
     * member field of this {@code IScope} object.The {@code name} parameter is
     * a {@code String} specifying the simple name of the desired field.
     *
     * @param name the field name.
     * @return the {@code IField} object of this class specified by
     * {@code name}.
     * @throws java.lang.NoSuchFieldException if the given field cannot be
     * found.
     */
    public Field getField(final String name) throws NoSuchFieldException;

    /**
     * Returns a {@code Map} containing all the fields defined in the current
     * scope. The map has the field name as key.
     *
     * @return a {@code Map} containing all the fields defined in the current
     * scope.
     */
    public Map<String, Field> getFields();

    /**
     * Returns a {@code IMethod} object that reflects the specified public
     * member method of the class or interface represented by this
     * {@code IScope} object.The {@code name} parameter is a {@code String}
     * specifying the simple name of the desired method. The
     * {@code parameterTypes} parameter is an array of {@code IType} objects
     * that identify the method's formal parameter types, in declared order. If
     * {@code parameterTypes} is {@code null}, it is treated as if it were an
     * empty array.
     *
     * @param name the name of the method.
     * @param parameter_types the list of parameters.
     * @return the {@code IMethod} object that matches the specified
     * {@code name} and {@code parameterTypes}.
     * @throws java.lang.NoSuchMethodException if the given method cannot be
     * found.
     */
    public default Method getMethod(final String name, final Type... parameter_types) throws NoSuchMethodException {
        assert Stream.of(parameter_types).noneMatch(Objects::isNull);
        return getScope().getMethod(name, parameter_types);
    }

    /**
     * Returns a collection containing {@code IMethod} objects reflecting all
     * the public methods of the class or interface represented by this {@code
     * IScope} object.
     *
     * @return the collection of {@code IMethod} objects representing the public
     * methods of this object.
     */
    public default Collection<Method> getMethods() {
        return getScope().getMethods();
    }

    /**
     * Returns a predicate whose scope is within this {@code IType} object
     * scope.If a predicate is not found within this scope, the enclosing scope
     * is checked.
     *
     * @param name the name of the predicate to find.
     * @return a predicate having the given name.
     * @throws java.lang.ClassNotFoundException if the given predicate cannot be
     * found.
     */
    public default Predicate getPredicate(final String name) throws ClassNotFoundException {
        return getScope().getPredicate(name);
    }

    /**
     * Returns all the predicates enclosed by this {@code IType} object mapped
     * by their names.
     *
     * @return a map containing all the predicates enclosed by this type having
     * type names as indexes.
     */
    public default Map<String, Predicate> getPredicates() {
        return getScope().getPredicates();
    }

    /**
     * Returns a type whose scope is within this {@code IScope} object scope.If
     * a type is not found within this scope, the enclosing scope is checked.
     *
     * @param name the name of the type to find.
     * @return a type having the given name.
     * @throws java.lang.ClassNotFoundException if the given type cannot be
     * found.
     */
    public default Type getType(final String name) throws ClassNotFoundException {
        return getScope().getType(name);
    }

    /**
     * Returns all the types enclosed by this {@code IScope} object mapped by
     * their names.
     *
     * @return a map containing all the types enclosed by this type having type
     * names as indexes.
     */
    public default Map<String, Type> getTypes() {
        return getScope().getTypes();
    }
}
