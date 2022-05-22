package it.cnr.istc.pst.oratio.utils;

import java.util.Collection;

import com.fasterxml.jackson.databind.ObjectMapper;

import it.cnr.istc.pst.oratio.Bound;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.timelines.Timeline;

public abstract class Message {

    public static final ObjectMapper MAPPER = new ObjectMapper();
    public final MessageType type;

    public Message(MessageType type) {
        this.type = type;
    }

    public static class Log extends Message {

        public final String log;

        public Log(final String log) {
            super(MessageType.Log);
            this.log = log;
        }
    }

    public static class StartedSolving extends Message {

        public StartedSolving() {
            super(MessageType.StartedSolving);
        }
    }

    public static class SolutionFound extends Message {

        public SolutionFound() {
            super(MessageType.SolutionFound);
        }
    }

    public static class InconsistentProblem extends Message {

        public InconsistentProblem() {
            super(MessageType.InconsistentProblem);
        }
    }

    public static class Graph extends Message {

        public final Collection<Flaw> flaws;
        public final Collection<Resolver> resolvers;

        public Graph(final Collection<Flaw> flaws, final Collection<Resolver> resolvers) {
            super(MessageType.Graph);
            this.flaws = flaws;
            this.resolvers = resolvers;
        }
    }

    public static class FlawCreated extends Message {

        public final long id;
        public final long[] causes;
        public final String label;
        public final byte state;
        public final Bound position;

        public FlawCreated(final long id, final long[] causes, final String label, final byte state,
                final Bound position) {
            super(MessageType.FlawCreated);
            this.id = id;
            this.causes = causes;
            this.label = label;
            this.state = state;
            this.position = position;
        }
    }

    public static class FlawStateChanged extends Message {

        public final long id;
        public final byte state;

        public FlawStateChanged(final long id, final byte state) {
            super(MessageType.FlawStateChanged);
            this.id = id;
            this.state = state;
        }
    }

    public static class FlawCostChanged extends Message {

        public final long id;
        public final Rational cost;

        public FlawCostChanged(final long id, final Rational cost) {
            super(MessageType.FlawCostChanged);
            this.id = id;
            this.cost = cost;
        }
    }

    public static class FlawPositionChanged extends Message {

        public final long id;
        public final Bound position;

        public FlawPositionChanged(final long id, final Bound position) {
            super(MessageType.FlawPositionChanged);
            this.id = id;
            this.position = position;
        }
    }

    public static class CurrentFlaw extends Message {

        public final long id;

        public CurrentFlaw(final long id) {
            super(MessageType.CurrentFlaw);
            this.id = id;
        }
    }

    public static class ResolverCreated extends Message {

        public final long id;
        public final long effect;
        public final Rational cost;
        public final String label;
        public final byte state;

        public ResolverCreated(final long id, final long effect, final Rational cost, final String label,
                final byte state) {
            super(MessageType.ResolverCreated);
            this.id = id;
            this.effect = effect;
            this.cost = cost;
            this.label = label;
            this.state = state;
        }
    }

    public static class ResolverStateChanged extends Message {

        public final long id;
        public final byte state;

        public ResolverStateChanged(final long id, final byte state) {
            super(MessageType.ResolverStateChanged);
            this.id = id;
            this.state = state;
        }
    }

    public static class CurrentResolver extends Message {

        public final long id;

        public CurrentResolver(final long id) {
            super(MessageType.CurrentResolver);
            this.id = id;
        }
    }

    public static class CausalLinkAdded extends Message {

        public final long flaw;
        public final long resolver;

        public CausalLinkAdded(final long flaw, final long resolver) {
            super(MessageType.CausalLinkAdded);
            this.flaw = flaw;
            this.resolver = resolver;
        }
    }

    public static class Timelines extends Message {

        public final Collection<Timeline<?>> timelines;

        public Timelines(final Collection<Timeline<?>> timelines) {
            super(MessageType.Timelines);
            this.timelines = timelines;
        }
    }

    public static class Tick extends Message {

        public final Rational current_time;

        public Tick(final Rational current_time) {
            super(MessageType.Tick);
            this.current_time = current_time;
        }
    }

    public static class StartingAtoms extends Message {

        public final long[] atoms;

        public StartingAtoms(final long[] atoms) {
            super(MessageType.StartingAtoms);
            this.atoms = atoms;
        }
    }

    public static class StartAtoms extends Message {

        public final long[] atoms;

        public StartAtoms(final long[] atoms) {
            super(MessageType.StartAtoms);
            this.atoms = atoms;
        }
    }

    public static class EndingAtoms extends Message {

        public final long[] atoms;

        public EndingAtoms(final long[] atoms) {
            super(MessageType.EndingAtoms);
            this.atoms = atoms;
        }
    }

    public static class EndAtoms extends Message {

        public final long[] atoms;

        public EndAtoms(final long[] atoms) {
            super(MessageType.EndAtoms);
            this.atoms = atoms;
        }
    }

    public enum MessageType {
        Log, StartedSolving, SolutionFound, InconsistentProblem, Graph, FlawCreated, FlawStateChanged, FlawCostChanged,
        FlawPositionChanged, CurrentFlaw, ResolverCreated, ResolverStateChanged, CurrentResolver, CausalLinkAdded,
        Timelines, Tick, StartingAtoms, StartAtoms, EndingAtoms, EndAtoms;
    }
}