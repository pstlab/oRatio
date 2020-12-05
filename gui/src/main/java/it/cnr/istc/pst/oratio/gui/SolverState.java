package it.cnr.istc.pst.oratio.gui;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import it.cnr.istc.pst.oratio.Context;
import it.cnr.istc.pst.oratio.StateListener;
import it.cnr.istc.pst.oratio.riddle.Core;
import it.cnr.istc.pst.oratio.timelines.ReusableResource;
import it.cnr.istc.pst.oratio.timelines.StateVariable;
import it.cnr.istc.pst.oratio.timelines.Timeline;

public class SolverState implements StateListener {

    private static final String TIMELINES = "timelines ";

    @Override
    public void log(String log) {
        // TODO Auto-generated method stub
    }

    @Override
    public void stateChanged(Core core) {
        App.broadcast(TIMELINES + App.GSON.toJson(getTimelines()));
    }

    Collection<Object> getTimelines() {
        Collection<Object> timelines = new ArrayList<>();
        for (Timeline<?> timeline : Context.getContext().getTimelines()) {
            if (timeline instanceof StateVariable) {
                StateVariable sv = (StateVariable) timeline;
                timelines
                        .add(new SVTimeline(
                                sv.getName(), sv.getOrigin().doubleValue(), sv.getHorizon().doubleValue(), sv
                                        .getValues().stream().map(
                                                val -> new SVTimeline.Value(
                                                        val.getAtoms().stream().map(atm -> atm.toString())
                                                                .collect(Collectors.joining(", ")),
                                                        val.getFrom().doubleValue(), val.getTo().doubleValue(),
                                                        val.getAtoms().stream().map(atm -> atm.getSigma())
                                                                .collect(Collectors.toList())))
                                        .collect(Collectors.toList())));
            } else if (timeline instanceof ReusableResource) {
                ReusableResource rr = (ReusableResource) timeline;
                timelines
                        .add(new RRTimeline(rr.getName(), rr.getCapacity().doubleValue(), rr.getOrigin().doubleValue(),
                                rr.getHorizon().doubleValue(),
                                rr.getValues().stream().map(val -> new RRTimeline.Value(val.getUsage().doubleValue(),
                                        val.getFrom().doubleValue(), val.getTo().doubleValue(), val.getAtoms().stream()
                                                .map(atm -> atm.getSigma()).collect(Collectors.toList())))
                                        .collect(Collectors.toList())));
            }
        }
        return timelines;
    }

    @Override
    public void init() {
        App.broadcast(TIMELINES + App.GSON.toJson(getTimelines()));
        App.GRAPH.clear();
    }

    @SuppressWarnings("unused")
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
}
