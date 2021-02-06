package it.cnr.istc.pst.oratio;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Stream;

public class Type implements Scope {

    final Solver solver;
    final Scope scope;
    final Map<String, Field> fields = new LinkedHashMap<>();
    final String name;
    final Collection<Type> superclasses = new ArrayList<>();
    final Collection<Constructor> constructors = new ArrayList<>();
    final Map<String, Collection<Method>> methods = new LinkedHashMap<>();
    final Map<String, Type> types = new LinkedHashMap<>();
    final Map<String, Predicate> predicates = new LinkedHashMap<>();
    final Collection<Item> instances = new ArrayList<>();

    Type(final Solver solver, final Scope scope, final String name) {
        this.solver = solver;
        this.scope = scope;
        this.name = name;
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
        Field field = fields.get(name);
        if (field != null)
            return field;

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

    @Override
    public Map<String, Field> getFields() {
        return Collections.unmodifiableMap(fields);
    }

    @SuppressWarnings("unused")
    private void defineField(Field field) {
        assert (!fields.containsKey(field.name));
        fields.put(field.name, field);
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

    public Constructor getConstructor(final Type... parameter_types) throws NoSuchMethodException {
        if (Stream.of(parameter_types).anyMatch(Objects::isNull))
            throw new IllegalArgumentException("parameter types cannot be null");
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

    public Collection<Constructor> getConstructors() {
        return Collections.unmodifiableCollection(constructors);
    }

    @SuppressWarnings("unused")
    private void defineConstructor(final Constructor constructor) {
        assert (!constructors.contains(constructor));
        constructors.add(constructor);
    }

    @Override
    public Method getMethod(final String name, final Type... parameter_types) throws NoSuchMethodException {
        if (Stream.of(parameter_types).anyMatch(Objects::isNull))
            throw new IllegalArgumentException("parameter types cannot be null");
        boolean isCorrect;
        if (methods.containsKey(name))
            for (Method m : methods.get(name))
                if (m.pars.length == parameter_types.length) {
                    isCorrect = true;
                    for (int i = 0; i < m.pars.length; i++)
                        if (!m.pars[i].type.isAssignableFrom(parameter_types[i])) {
                            isCorrect = false;
                            break;
                        }
                    if (isCorrect)
                        return m;
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
        methods.values().forEach(ms -> c_methods.addAll(ms));
        return Collections.unmodifiableCollection(c_methods);
    }

    @SuppressWarnings("unused")
    private void defineMethod(final Method method) {
        if (!methods.containsKey(method.name))
            methods.put(method.name, new ArrayList<>());
        assert (!methods.get(method.name).contains(method));
        methods.get(method.name).add(method);
    }

    @Override
    public Type getType(String name) throws ClassNotFoundException {
        Type type = types.get(name);
        if (type != null)
            return type;

        // if not here, check any enclosing scope
        try {
            return scope.getType(name);
        } catch (ClassNotFoundException e) {
            for (Type superclass : superclasses) {
                try {
                    return superclass.getType(name);
                } catch (ClassNotFoundException ex) {
                }
            }
        }

        // not found
        throw new ClassNotFoundException(name);
    }

    @Override
    public Map<String, Type> getTypes() {
        return Collections.unmodifiableMap(types);
    }

    @SuppressWarnings("unused")
    private void defineType(Type type) {
        assert (!types.containsKey(type.name));
        types.put(type.name, type);
    }

    @Override
    public Predicate getPredicate(final String name) throws ClassNotFoundException {
        Predicate predicate = predicates.get(name);
        if (predicate != null)
            return predicate;

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

    @SuppressWarnings("unused")
    private void definePredicate(Predicate predicate) {
        assert (!predicates.containsKey(predicate.name));
        predicates.put(predicate.name, predicate);
    }
}
