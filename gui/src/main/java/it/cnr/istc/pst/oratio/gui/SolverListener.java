package it.cnr.istc.pst.oratio.gui;

import static it.cnr.istc.pst.oratio.gui.App.MAPPER;

import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Stream;

import com.fasterxml.jackson.core.JsonProcessingException;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.oratio.Bound;
import it.cnr.istc.pst.oratio.GraphListener;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.StateListener;
import it.cnr.istc.pst.oratio.timelines.ExecutorListener;
import it.cnr.istc.pst.oratio.timelines.TimelinesExecutor;
import it.cnr.istc.pst.oratio.utils.Flaw;
import it.cnr.istc.pst.oratio.utils.Message;
import it.cnr.istc.pst.oratio.utils.Resolver;

public class SolverListener implements StateListener, GraphListener, ExecutorListener {

    private static final Logger LOG = LoggerFactory.getLogger(SolverListener.class);
    private final Solver solver;
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
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.Timelines(solver.getTimelines())));
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
        state = SolverState.Solved;
        if (current_flaw != null)
            current_flaw.current = false;
        current_flaw = null;
        if (current_resolver != null)
            current_resolver.current = false;
        current_resolver = null;
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.Timelines(solver.getTimelines())));
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
    public synchronized void startAtoms(final long[] atoms) {
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.StartAtoms(atoms)));
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

    @Override
    public synchronized void endAtoms(final long[] atoms) {
        try {
            App.broadcast(MAPPER.writeValueAsString(new Message.EndAtoms(atoms)));
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

    public enum SolverState {
        Idle, Solving, Solved, Inconsistent;
    }
}
