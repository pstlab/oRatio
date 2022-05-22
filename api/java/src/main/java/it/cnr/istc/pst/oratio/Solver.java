package it.cnr.istc.pst.oratio;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Stream;

import it.cnr.istc.pst.oratio.GraphListener.State;
import it.cnr.istc.pst.oratio.timelines.Timeline;

public class Solver implements Scope, Env {

    static {
        System.loadLibrary("json");
        System.loadLibrary("smt");
        System.loadLibrary("riddle");
        System.loadLibrary("core");
        System.loadLibrary("solver");
        System.loadLibrary("executor");
        System.loadLibrary("solver-java-api");
        System.loadLibrary("executor-java-api");
    }
    public static final String BOOL = "bool";
    public static final String INT = "int";
    public static final String REAL = "real";
    public static final String TP = "tp";
    public static final String STRING = "string";
    public static final String IMPULSE = "Impulse";
    public static final String INTERVAL = "Interval";
    public static final String ORIGIN = "origin";
    public static final String HORIZON = "horizon";
    public static final String AT = "at";
    public static final String START = "start";
    public static final String END = "end";

    private final long native_handle;
    final Map<String, Field> fields = new LinkedHashMap<>();
    final Map<String, Collection<Method>> methods = new LinkedHashMap<>();
    final Map<String, Type> types = new LinkedHashMap<>();
    final Map<String, Predicate> predicates = new LinkedHashMap<>();
    final Map<String, Item> exprs = new LinkedHashMap<>();
    private final TimelinesList timelines;
    private final boolean build_timelines_at_state_change;
    private final boolean build_timelines_at_solution_found;
    private final Collection<GraphListener> graph_listeners = new ArrayList<>();
    private final Collection<StateListener> state_listeners = new ArrayList<>();

    public Solver() {
        this(false, true);
    }

    public Solver(final boolean build_timelines_at_state_change, final boolean build_timelines_at_solution_found) {
        native_handle = new_instance();
        this.build_timelines_at_state_change = build_timelines_at_state_change;
        this.build_timelines_at_solution_found = build_timelines_at_solution_found;
        timelines = build_timelines_at_state_change || build_timelines_at_solution_found ? new TimelinesList(this)
                : null;
    }

    private native long new_instance();

    public native void dispose();

    @Override
    public Solver getSolver() {
        return this;
    }

    @Override
    public Scope getScope() {
        return this;
    }

    @Override
    public Field getField(final String name) throws NoSuchFieldException {
        return fields.get(name);
    }

    @Override
    public Map<String, Field> getFields() {
        return Collections.unmodifiableMap(fields);
    }

    @SuppressWarnings("unused")
    private void defineField(final Field field) {
        assert (!fields.containsKey(field.name));
        fields.put(field.name, field);
    }

    @Override
    public Method getMethod(final String name, final Type... parameter_types) throws NoSuchMethodException {
        if (Stream.of(parameter_types).anyMatch(Objects::isNull))
            throw new IllegalArgumentException("parameter types cannot be null");
        boolean isCorrect;
        if (methods.containsKey(name))
            for (final Method m : methods.get(name))
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

        // not found
        throw new NoSuchMethodException(name);
    }

    @Override
    public Collection<Method> getMethods() {
        final Collection<Method> c_methods = new ArrayList<>(methods.size());
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
    public Type getType(final String name) throws ClassNotFoundException {
        final Type type = types.get(name);
        if (type != null)
            return type;

        // not found
        throw new ClassNotFoundException(name);
    }

    @Override
    public Map<String, Type> getTypes() {
        return Collections.unmodifiableMap(types);
    }

    @SuppressWarnings("unused")
    private void defineType(final Type type) {
        assert (!types.containsKey(type.name));
        types.put(type.name, type);
    }

    @Override
    public Predicate getPredicate(final String name) throws ClassNotFoundException {
        final Predicate predicate = predicates.get(name);
        if (predicate != null)
            return predicate;

        // not found
        throw new ClassNotFoundException(name);
    }

    @Override
    public Map<String, Predicate> getPredicates() {
        return Collections.unmodifiableMap(predicates);
    }

    @SuppressWarnings("unused")
    private void definePredicate(final Predicate predicate) {
        assert (!predicates.containsKey(predicate.name));
        predicates.put(predicate.name, predicate);
    }

    @Override
    public Item get(final String name) throws NoSuchFieldException {
        final Item item = exprs.get(name);
        if (item != null)
            return item;

        // not found
        throw new NoSuchFieldException(name);
    }

    @Override
    public Map<String, Item> getExprs() {
        return Collections.unmodifiableMap(exprs);
    }

    @SuppressWarnings("unused")
    private void set(final String id, final Item itm) {
        exprs.put(id, itm);
    }

    public Collection<Timeline<?>> getTimelines() {
        return Collections.unmodifiableList(timelines);
    }

    public native void read(String script) throws SolverException;

    public native void read(String[] files) throws SolverException;

    public native void solve() throws SolverException;

    private void fireFlawCreated(final long id, final long[] causes, final String label, final byte state,
            final int position_lb, final int position_ub) {
        final State c_state = State.values()[state];
        final Bound position = new Bound(position_lb, position_ub);
        graph_listeners.stream().forEach(l -> l.flawCreated(id, causes, label, c_state, position));
    }

    private void fireFlawStateChanged(final long id, final byte state) {
        final State c_state = State.values()[state];
        graph_listeners.stream().forEach(l -> l.flawStateChanged(id, c_state));
    }

    private void fireFlawCostChanged(final long id, final long cost_num, final long cost_den) {
        final Rational cost = new Rational(cost_num, cost_den);
        graph_listeners.stream().forEach(l -> l.flawCostChanged(id, cost));
    }

    private void fireFlawPositionChanged(final long id, final int position_lb, final int position_ub) {
        final Bound position = new Bound(position_lb, position_ub);
        graph_listeners.stream().forEach(l -> l.flawPositionChanged(id, position));
    }

    private void fireCurrentFlaw(final long id) {
        graph_listeners.stream().forEach(l -> l.currentFlaw(id));
    }

    private void fireResolverCreated(final long id, final long effect, final String label, final long cost_num,
            final long cost_den, final byte state) {
        final State c_state = State.values()[state];
        final Rational cost = new Rational(cost_num, cost_den);
        graph_listeners.stream().forEach(l -> l.resolverCreated(id, effect, cost, label, c_state));
    }

    private void fireResolverStateChanged(final long id, final byte state) {
        final State c_state = State.values()[state];
        graph_listeners.stream().forEach(l -> l.resolverStateChanged(id, c_state));
    }

    private void fireCurrentResolver(final long id) {
        graph_listeners.stream().forEach(l -> l.currentResolver(id));
    }

    private void fireCausalLinkAdded(final long flaw, final long resolver) {
        graph_listeners.stream().forEach(l -> l.causalLinkAdded(flaw, resolver));
    }

    private void fireLog(final String log) {
        state_listeners.stream().forEach(l -> l.log(log));
    }

    private void fireRead(final String script) {
        state_listeners.stream().forEach(l -> l.read(script));
    }

    private void fireRead(final String[] files) {
        state_listeners.stream().forEach(l -> l.read(files));
    }

    private void fireStateChanged() {
        if (build_timelines_at_state_change)
            timelines.stateChanged();
        state_listeners.stream().forEach(l -> l.stateChanged());
    }

    private void fireStartedSolving() {
        state_listeners.stream().forEach(l -> l.startedSolving());
    }

    private void fireSolutionFound() {
        if (build_timelines_at_solution_found)
            timelines.stateChanged();
        state_listeners.stream().forEach(l -> l.solutionFound());
    }

    private void fireInconsistentProblem() {
        state_listeners.stream().forEach(l -> l.inconsistentProblem());
    }

    public void addGraphListener(final GraphListener l) {
        graph_listeners.add(l);
    }

    public void removeGraphListener(final GraphListener l) {
        graph_listeners.remove(l);
    }

    public void addStateListener(final StateListener l) {
        state_listeners.add(l);
    }

    public void removeStateListener(final StateListener l) {
        state_listeners.remove(l);
    }
}
