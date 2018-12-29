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

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Stream;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class Type extends BaseScope {

    final String name;
    final Collection<Type> superclasses = new ArrayList<>();
    final Collection<Constructor> constructors = new ArrayList<>();
    final Map<String, Collection<Method>> methods = new HashMap<>();
    final Map<String, Type> types = new LinkedHashMap<>();
    final Map<String, Predicate> predicates = new LinkedHashMap<>();
    final Collection<Item> instances = new ArrayList<>();

    Type(final Core core, final Scope scope, final String name) {
        super(core, scope);
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public Collection<Item> getInstances() {
        return Collections.unmodifiableCollection(instances);
    }

    public boolean isAssignableFrom(final Type type) {
        LinkedList<Type> queue = new LinkedList<>();
        queue.add(type);
        while (!queue.isEmpty()) {
            Type c_type = queue.pollFirst();
            if (c_type == this) {
                return true;
            }
            queue.addAll(c_type.superclasses);
        }
        return false;
    }

    public Collection<Type> getSuperclasses() {
        return Collections.unmodifiableCollection(superclasses);
    }

    @Override
    public Field getField(final String name) throws NoSuchFieldException {
        Field field = fields.get(name);
        if (field != null) {
            return field;
        }

        // if not here, check any enclosing scope
        try {
            return scope.getField(name);
        } catch (NoSuchFieldException e) {
            // if not in any enclosing scope, check any superclass
            for (Type superclass : superclasses) {
                try {
                    return superclass.getField(name);
                } catch (NoSuchFieldException ex) {
                }
            }
        }

        // not found
        throw new NoSuchFieldException(name);
    }

    public Constructor getConstructor(final Type... parameter_types) throws NoSuchMethodException {
        assert Stream.of(parameter_types).noneMatch(Objects::isNull);
        boolean is_correct;
        for (Constructor c : constructors) {
            if (c.pars.length == parameter_types.length) {
                is_correct = true;
                for (int i = 0; i < c.pars.length; i++) {
                    if (!c.pars[i].getType().isAssignableFrom(parameter_types[i])) {
                        is_correct = false;
                        break;
                    }
                }
                if (is_correct) {
                    return c;
                }
            }
        }

        // not found
        throw new NoSuchMethodException(name);
    }

    @Override
    public Method getMethod(final String name, final Type... parameter_types) throws NoSuchMethodException {
        assert Stream.of(parameter_types).noneMatch(Objects::isNull);
        boolean isCorrect;
        if (methods.containsKey(name)) {
            for (Method m : methods.get(name)) {
                if (m.pars.length == parameter_types.length) {
                    isCorrect = true;
                    for (int i = 0; i < m.pars.length; i++) {
                        if (!m.pars[i].type.isAssignableFrom(parameter_types[i])) {
                            isCorrect = false;
                            break;
                        }
                    }
                    if (isCorrect) {
                        return m;
                    }
                }
            }
        }

        try {
            // if not here, check any enclosing scope
            return scope.getMethod(name, parameter_types);
        } catch (NoSuchMethodException e) {
            // if not in any enclosing scope, check any superclass
            for (Type superclass : superclasses) {
                try {
                    return superclass.getMethod(name, parameter_types);
                } catch (NoSuchMethodException ex) {
                }
            }
        }

        // not found
        throw new NoSuchMethodException(name);
    }

    @Override
    public Collection<Method> getMethods() {
        Collection<Method> c_methods = new ArrayList<>(methods.size());
        methods.values().forEach((ms) -> {
            c_methods.addAll(ms);
        });
        return Collections.unmodifiableCollection(c_methods);
    }

    void defineMethod(final Method method) {
        if (!methods.containsKey(method.name)) {
            methods.put(method.name, new ArrayList<>());
        }
        methods.get(method.name).add(method);
    }

    @Override
    public Predicate getPredicate(final String name) throws ClassNotFoundException {
        Predicate predicate = predicates.get(name);
        if (predicate != null) {
            return predicate;
        }

        // if not here, check any enclosing scope
        try {
            return scope.getPredicate(name);
        } catch (ClassNotFoundException e) {
            // if not in any enclosing scope, check any superclass
            for (Type superclass : superclasses) {
                try {
                    return superclass.getPredicate(name);
                } catch (ClassNotFoundException ex) {
                }
            }
        }

        // not found
        throw new ClassNotFoundException(name);
    }

    @Override
    public Map<String, Predicate> getPredicates() {
        return Collections.unmodifiableMap(predicates);
    }
}
