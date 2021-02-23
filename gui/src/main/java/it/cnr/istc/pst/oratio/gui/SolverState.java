package it.cnr.istc.pst.oratio.gui;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonAutoDetect.Visibility;
import com.fasterxml.jackson.core.JsonProcessingException;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.StateListener;
import it.cnr.istc.pst.oratio.timelines.PropositionalAgent;
import it.cnr.istc.pst.oratio.timelines.ReusableResource;
import it.cnr.istc.pst.oratio.timelines.StateVariable;
import it.cnr.istc.pst.oratio.timelines.Timeline;
import it.cnr.istc.pst.oratio.timelines.TimelinesList;

public class SolverState implements StateListener {

    private static final Logger LOG = LoggerFactory.getLogger(SolverState.class);
    private final Solver solver;
    private final TimelinesList timelines;

    public SolverState(final Solver solver) {
        this.solver = solver;
        timelines = new TimelinesList(solver);
    }

    public Solver getSolver() {
        return solver;
    }

    @Override
    public void log(String log) {
        try {
            App.broadcast(App.MAPPER.writeValueAsString(new App.Message.Log(log)));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void read(String arg0) {
    }

    @Override
    public void read(String[] arg0) {
    }

    @Override
    public void stateChanged() {
        timelines.stateChanged();
        try {
            App.broadcast(App.MAPPER.writeValueAsString(new App.Message.Timelines(getTimelines())));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    Collection<Object> getTimelines() {
        Collection<Object> c_tls = new ArrayList<>();
        for (Timeline<?> tl : timelines) {
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

    private static SVTimeline toTimeline(StateVariable sv) {
        return new SVTimeline(sv.getName(), sv.getOrigin().doubleValue(), sv.getHorizon().doubleValue(),
                sv.getValues().stream()
                        .map(val -> new SVTimeline.Value(
                                val.getAtoms().stream().map(atm -> atm.toString()).collect(Collectors.joining(", ")),
                                val.getFrom().doubleValue(), val.getTo().doubleValue(),
                                val.getAtoms().stream().map(atm -> atm.getSigma()).collect(Collectors.toList())))
                        .collect(Collectors.toList()));
    }

    private static RRTimeline toTimeline(ReusableResource rr) {
        return new RRTimeline(rr.getName(), rr.getCapacity().doubleValue(), rr.getOrigin().doubleValue(),
                rr.getHorizon().doubleValue(),
                rr.getValues().stream()
                        .map(val -> new RRTimeline.Value(val.getUsage().doubleValue(), val.getFrom().doubleValue(),
                                val.getTo().doubleValue(),
                                val.getAtoms().stream().map(atm -> atm.getSigma()).collect(Collectors.toList())))
                        .collect(Collectors.toList()));
    }

    private static Agent toTimeline(PropositionalAgent pa) {
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
}
