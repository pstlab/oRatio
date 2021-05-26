package it.cnr.istc.pst.oratio.utils;

import java.util.List;
import java.util.stream.Collectors;

import com.fasterxml.jackson.core.JsonProcessingException;

import it.cnr.istc.pst.oratio.timelines.PropositionalAgent;
import it.cnr.istc.pst.oratio.timelines.ReusableResource;
import it.cnr.istc.pst.oratio.timelines.StateVariable;

public class Timeline {

    public final TimelineType type;
    public final String name;
    public final double origin, horizon;

    public Timeline(final TimelineType type, final String name, final double origin, final double horizon) {
        this.type = type;
        this.name = name;
        this.origin = origin;
        this.horizon = horizon;
    }

    public static class SVTimeline extends Timeline {

        public final List<Value> values;

        public SVTimeline(final StateVariable sv) {
            this(sv.getName(), sv.getOrigin().doubleValue(), sv.getHorizon().doubleValue(),
                    sv.getValues().stream().map(val -> {
                        try {
                            return new SVTimeline.Value(Message.MAPPER.writeValueAsString(val.getAtoms()),
                                    val.getFrom().doubleValue(), val.getTo().doubleValue());
                        } catch (JsonProcessingException e) {
                            return null;
                        }
                    }).collect(Collectors.toList()));
        }

        public SVTimeline(final String name, final double origin, final double horizon, final List<Value> values) {
            super(TimelineType.StateVariable, name, origin, horizon);
            this.values = values;
        }

        public static class Value {

            public final String value;
            public final double from, to;

            public Value(final String value, final double from, final double to) {
                this.value = value;
                this.from = from;
                this.to = to;
            }
        }
    }

    public static class RRTimeline extends Timeline {

        public final double capacity;
        public final List<Value> values;

        public RRTimeline(ReusableResource rr) {
            this(rr.getName(), rr.getCapacity().doubleValue(), rr.getOrigin().doubleValue(),
                    rr.getHorizon().doubleValue(), rr.getValues().stream().map(val -> {
                        try {
                            return new RRTimeline.Value(Message.MAPPER.writeValueAsString(val.getAtoms()),
                                    val.getUsage().doubleValue(), val.getFrom().doubleValue(),
                                    val.getTo().doubleValue());
                        } catch (JsonProcessingException e) {
                            return null;
                        }
                    }).collect(Collectors.toList()));
        }

        public RRTimeline(final String name, final double capacity, final double origin, final double horizon,
                final List<Value> values) {
            super(TimelineType.ReusableResource, name, origin, horizon);
            this.capacity = capacity;
            this.values = values;
        }

        public static class Value {

            public final String value;
            public final double usage;
            public final double from, to;

            public Value(final String value, final double usage, final double from, final double to) {
                this.value = value;
                this.usage = usage;
                this.from = from;
                this.to = to;
            }
        }
    }

    public static class Agent extends Timeline {

        public final List<Value> values;

        public Agent(PropositionalAgent pa) {
            this(pa.getName(), pa.getOrigin().doubleValue(), pa.getHorizon().doubleValue(),
                    pa.getValues().stream().map(val -> {
                        try {
                            return new Agent.Value(Message.MAPPER.writeValueAsString(val.getAtom()),
                                    val.getFrom().doubleValue(), val.getTo().doubleValue());
                        } catch (JsonProcessingException e) {
                            return null;
                        }
                    }).collect(Collectors.toList()));
        }

        public Agent(final String name, final double origin, final double horizon, final List<Value> values) {
            super(TimelineType.Agent, name, origin, horizon);
            this.values = values;
        }

        public static class Value {

            public final String value;
            public final double from, to;

            public Value(final String value, final double from, final double to) {
                this.value = value;
                this.from = from;
                this.to = to;
            }
        }
    }

    public enum TimelineType {
        StateVariable, ReusableResource, Agent;
    }
}
