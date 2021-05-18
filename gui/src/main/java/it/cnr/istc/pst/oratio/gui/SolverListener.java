package it.cnr.istc.pst.oratio.gui;

import static it.cnr.istc.pst.oratio.gui.App.MAPPER;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonAutoDetect.Visibility;
import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.oratio.Bound;
import it.cnr.istc.pst.oratio.GraphListener;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.StateListener;
import it.cnr.istc.pst.oratio.gui.App.Message;
import it.cnr.istc.pst.oratio.timelines.ExecutorListener;
import it.cnr.istc.pst.oratio.timelines.PropositionalAgent;
import it.cnr.istc.pst.oratio.timelines.ReusableResource;
import it.cnr.istc.pst.oratio.timelines.StateVariable;
import it.cnr.istc.pst.oratio.timelines.Timeline;
import it.cnr.istc.pst.oratio.timelines.TimelinesExecutor;
import it.cnr.istc.pst.oratio.timelines.TimelinesList;

public class SolverListener implements StateListener, GraphListener, ExecutorListener {

    private static final Logger LOG = LoggerFactory.getLogger(SolverListener.class);
    private final Solver solver;
    private final TimelinesList timelines;
    private final TimelinesExecutor executor;
    private Rational current_time = new Rational();
    private final Map<Long, Flaw> flaws = new HashMap<>();
    private Flaw current_flaw = null;
    private final Map<Long, Resolver> resolvers = new HashMap<>();
    private Resolver current_resolver = null;
    private SolverState state = SolverState.Idle;

    public SolverListener(final Solver solver, final TimelinesExecutor executor) {
        this.solver = solver;
        this.executor = executor;
        timelines = new TimelinesList(solver);
    }

    public Solver getSolver() {
        return solver;
    }

    public TimelinesExecutor getExecutor() {
        return executor;
    }

    public SolverState getState() {
        return state;
    }

    @Override
    public synchronized void log(final String log) {
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.Log(log)));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void read(final String arg0) {
    }

    @Override
    public synchronized void read(final String[] arg0) {
    }

    @Override
    public synchronized void stateChanged() {
        timelines.stateChanged();
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.Timelines(getTimelines())));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void startedSolving() {
        state = SolverState.Solving;
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.StartedSolving()));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void solutionFound() {
        timelines.stateChanged();
        state = SolverState.Solved;
        if (current_flaw != null)
            current_flaw.current = false;
        current_flaw = null;
        if (current_resolver != null)
            current_resolver.current = false;
        current_resolver = null;
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.Timelines(getTimelines())));
            App.broadcast(MAPPER.writeValueAsString(new Message.SolutionFound()));
            App.broadcast(MAPPER.writeValueAsString(new Message.Tick(current_time)));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void inconsistentProblem() {
        state = SolverState.Inconsistent;
        if (current_flaw != null)
            current_flaw.current = false;
        current_flaw = null;
        if (current_resolver != null)
            current_resolver.current = false;
        current_resolver = null;
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.InconsistentProblem()));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void flawCreated(final long id, final long[] causes, final String label, final State state,
            final Bound position) {
        final Flaw c_flaw = new Flaw(id,
                Arrays.stream(causes).mapToObj(r_id -> resolvers.get(r_id)).toArray(Resolver[]::new), label, state,
                position);
        Stream.of(c_flaw.causes).forEach(c -> c.preconditions.add(c_flaw));
        flaws.put(id, c_flaw);
        try {
            App.broadcast(MAPPER
                    .writeValueAsString(new Message.FlawCreated(id, causes, label, (byte) state.ordinal(), position)));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void flawStateChanged(final long id, final State state) {
        final Flaw flaw = flaws.get(id);
        flaw.state = state;
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.FlawStateChanged(id, (byte) state.ordinal())));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void flawCostChanged(final long id, final Rational cost) {
        final Flaw flaw = flaws.get(id);
        flaw.cost = cost;
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.FlawCostChanged(id, cost)));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void flawPositionChanged(final long id, final Bound position) {
        final Flaw flaw = flaws.get(id);
        flaw.position = position;
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.FlawPositionChanged(id, position)));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void currentFlaw(final long id) {
        if (current_flaw != null)
            current_flaw.current = false;
        final Flaw flaw = flaws.get(id);
        current_flaw = flaw;
        current_flaw.current = true;
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.CurrentFlaw(id)));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void resolverCreated(final long id, final long effect, final Rational cost, final String label,
            final State state) {
        final Resolver resolver = new Resolver(id, flaws.get(effect), label, state, cost);
        resolvers.put(id, resolver);
        try {
            App.broadcast(MAPPER
                    .writeValueAsString(new Message.ResolverCreated(id, effect, cost, label, (byte) state.ordinal())));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void resolverStateChanged(final long id, final State state) {
        final Resolver resolver = resolvers.get(id);
        resolver.state = state;
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.ResolverStateChanged(id, (byte) state.ordinal())));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void currentResolver(final long id) {
        if (current_resolver != null)
            current_resolver.current = false;
        final Resolver resolver = resolvers.get(id);
        current_resolver = resolver;
        current_resolver.current = true;
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.CurrentResolver(id)));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void causalLinkAdded(final long flaw, final long resolver) {
        resolvers.get(resolver).preconditions.add(flaws.get(flaw));
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.CausalLinkAdded(flaw, resolver)));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void tick(final Rational current_time) {
        this.current_time = current_time;
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.Tick(current_time)));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void startingAtoms(final long[] atoms) {
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.StartingAtoms(atoms)));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public synchronized void endingAtoms(final long[] atoms) {
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.EndingAtoms(atoms)));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    void clear() {
        flaws.clear();
        resolvers.clear();
        current_flaw = null;
        current_resolver = null;
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.Graph(flaws.values(), resolvers.values())));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    public Collection<Flaw> getFlaws() {
        return flaws.values();
    }

    public Flaw getCurrentFlaw() {
        return current_flaw;
    }

    public Collection<Resolver> getResolvers() {
        return resolvers.values();
    }

    public Resolver getCurrentResolver() {
        return current_resolver;
    }

    public Rational getCurrentTime() {
        return current_time;
    }

    Collection<Object> getTimelines() {
        final Collection<Object> c_tls = new ArrayList<>();
        for (final Timeline<?> tl : timelines) {
            if (tl instanceof StateVariable) {
                c_tls.add(toTimeline((StateVariable) tl));
            } else if (tl instanceof ReusableResource) {
                c_tls.add(toTimeline((ReusableResource) tl));
            } else if (tl instanceof PropositionalAgent) {
                c_tls.add(toTimeline((PropositionalAgent) tl));
            }
        }
        return c_tls;
    }

    private static SVTimeline toTimeline(final StateVariable sv) {
        return new SVTimeline(sv.getName(), sv.getOrigin().doubleValue(), sv.getHorizon().doubleValue(),
                sv.getValues().stream()
                        .map(val -> new SVTimeline.Value(
                                val.getAtoms().stream().map(atm -> atm.toString()).collect(Collectors.joining(", ")),
                                val.getFrom().doubleValue(), val.getTo().doubleValue(),
                                val.getAtoms().stream().map(atm -> atm.getSigma()).collect(Collectors.toList())))
                        .collect(Collectors.toList()));
    }

    private static RRTimeline toTimeline(final ReusableResource rr) {
        return new RRTimeline(rr.getName(), rr.getCapacity().doubleValue(), rr.getOrigin().doubleValue(),
                rr.getHorizon().doubleValue(),
                rr.getValues().stream()
                        .map(val -> new RRTimeline.Value(val.getUsage().doubleValue(), val.getFrom().doubleValue(),
                                val.getTo().doubleValue(),
                                val.getAtoms().stream().map(atm -> atm.getSigma()).collect(Collectors.toList())))
                        .collect(Collectors.toList()));
    }

    private static Agent toTimeline(final PropositionalAgent pa) {
        return new Agent(pa.getName(), pa.getOrigin().doubleValue(), pa.getHorizon().doubleValue(),
                pa.getValues().stream().map(val -> new Agent.Value(val.getAtom().toString(),
                        val.getFrom().doubleValue(), val.getTo().doubleValue(), val.getAtom().getSigma()))
                        .collect(Collectors.toList()));
    }

    @SuppressWarnings("unused")
    @JsonAutoDetect(fieldVisibility = Visibility.ANY)
    private static class SVTimeline {

        private final String type = "state-variable";
        private final String name;
        private final double origin, horizon;
        private final List<Value> values;

        private SVTimeline(final String name, final double origin, final double horizon, final List<Value> values) {
            this.name = name;
            this.origin = origin;
            this.horizon = horizon;
            this.values = values;
        }

        @JsonAutoDetect(fieldVisibility = Visibility.ANY)
        private static class Value {

            private final String name;
            private final double from, to;
            private final Collection<Long> atoms;

            private Value(final String name, final double from, final double to, final Collection<Long> atoms) {
                this.name = name;
                this.from = from;
                this.to = to;
                this.atoms = atoms;
            }
        }
    }

    @SuppressWarnings("unused")
    @JsonAutoDetect(fieldVisibility = Visibility.ANY)
    private static class RRTimeline {

        private final String type = "reusable-resource";
        private final String name;
        private final double capacity;
        private final double origin, horizon;
        private final List<Value> values;

        private RRTimeline(final String name, final double capacity, final double origin, final double horizon,
                final List<Value> values) {
            this.name = name;
            this.capacity = capacity;
            this.origin = origin;
            this.horizon = horizon;
            this.values = values;
        }

        @JsonAutoDetect(fieldVisibility = Visibility.ANY)
        private static class Value {

            private final double usage;
            private final double from, to;
            private final Collection<Long> atoms;

            private Value(final double usage, final double from, final double to, final Collection<Long> atoms) {
                this.usage = usage;
                this.from = from;
                this.to = to;
                this.atoms = atoms;
            }
        }
    }

    @SuppressWarnings("unused")
    @JsonAutoDetect(fieldVisibility = Visibility.ANY)
    private static class Agent {

        private final String type = "agent";
        private final String name;
        private final double origin, horizon;
        private final List<Value> values;

        private Agent(final String name, final double origin, final double horizon, final List<Value> values) {
            this.name = name;
            this.origin = origin;
            this.horizon = horizon;
            this.values = values;
        }

        @JsonAutoDetect(fieldVisibility = Visibility.ANY)
        private static class Value {

            private final String name;
            private final double from, to;
            private final Long atom;

            private Value(final String name, final double from, final double to, final Long atom) {
                this.name = name;
                this.from = from;
                this.to = to;
                this.atom = atom;
            }
        }
    }

    @JsonSerialize(using = FlawSerializer.class)
    class Flaw {

        private final long id;
        private final Resolver[] causes;
        private final String label;
        private State state;
        private Bound position;
        private Rational cost = Rational.POSITIVE_INFINITY;
        private boolean current = false;

        private Flaw(final long id, final Resolver[] causes, final String label, final State state,
                final Bound position) {
            this.id = id;
            this.causes = causes;
            this.label = label;
            this.state = state;
            this.position = position;
        }
    }

    private static class FlawSerializer extends StdSerializer<Flaw> {

        private static final long serialVersionUID = 1L;

        private FlawSerializer() {
            super(Flaw.class);
        }

        @Override
        public void serialize(final Flaw flaw, final JsonGenerator gen, final SerializerProvider provider)
                throws IOException {
            gen.writeStartObject();
            gen.writeNumberField("id", flaw.id);
            gen.writeArrayFieldStart("causes");
            Arrays.stream(flaw.causes).forEach(c -> {
                try {
                    gen.writeNumber(c.id);
                } catch (final IOException e) {
                    LOG.error("Cannot serialize", e);
                }
            });
            gen.writeEndArray();
            gen.writeStringField("label", flaw.label);
            gen.writeNumberField("state", flaw.state.ordinal());

            if (flaw.position.min != -Bound.INF || flaw.position.max != Bound.INF)
                gen.writeObjectField("position", flaw.position);

            gen.writeObjectField("cost", flaw.cost);

            gen.writeBooleanField("current", flaw.current);

            gen.writeEndObject();
        }
    }

    @JsonSerialize(using = ResolverSerializer.class)
    class Resolver {

        private final long id;
        private final Flaw effect;
        private final String label;
        private State state;
        private final Rational cost;
        private final Set<Flaw> preconditions = new HashSet<>();
        private boolean current = false;

        private Resolver(final long id, final Flaw effect, final String label, final State state, final Rational cost) {
            this.id = id;
            this.effect = effect;
            this.label = label;
            this.state = state;
            this.cost = cost;
        }
    }

    private static class ResolverSerializer extends StdSerializer<Resolver> {

        private ResolverSerializer() {
            super(Resolver.class);
        }

        /**
         *
         */
        private static final long serialVersionUID = 1L;

        @Override
        public void serialize(final Resolver resolver, final JsonGenerator gen, final SerializerProvider provider)
                throws IOException {
            gen.writeStartObject();
            gen.writeNumberField("id", resolver.id);
            gen.writeNumberField("effect", resolver.effect.id);
            gen.writeStringField("label", resolver.label);
            gen.writeNumberField("state", resolver.state.ordinal());

            gen.writeObjectField("cost", resolver.cost);

            gen.writeArrayFieldStart("preconditions");
            resolver.preconditions.stream().forEach(pre -> {
                try {
                    gen.writeNumber(pre.id);
                } catch (final IOException e) {
                    LOG.error("Cannot serialize", e);
                }
            });
            gen.writeEndArray();

            gen.writeBooleanField("current", resolver.current);

            gen.writeEndObject();
        }
    }

    public enum SolverState {
        Idle, Solving, Solved, Inconsistent;
    }
}
