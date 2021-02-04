package it.cnr.istc.pst.oratio.gui;

import java.io.IOException;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.stream.Stream;

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
import it.cnr.istc.pst.oratio.gui.App.Message;

public class CausalGraph implements GraphListener {

    private static final Logger LOG = LoggerFactory.getLogger(CausalGraph.class);
    private final Solver solver;
    private final Map<Long, Flaw> flaws = new HashMap<>();
    private Flaw current_flaw = null;
    private final Map<Long, Resolver> resolvers = new HashMap<>();
    private Resolver current_resolver = null;

    public CausalGraph(Solver solver) {
        this.solver = solver;
    }

    public Solver getSolver() {
        return solver;
    }

    void clear() {
        flaws.clear();
        resolvers.clear();
        current_flaw = null;
        current_resolver = null;
        try {
            App.broadcast(App.MAPPER.writeValueAsString(new Message.Graph(this)));
        } catch (JsonProcessingException e) {
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

    public void flawCreated(final long id, final long[] causes, final String label, final State state,
            final Bound position) {

        Flaw c_flaw = new Flaw(id, Arrays.stream(causes).mapToObj(r_id -> resolvers.get(r_id)).toArray(Resolver[]::new),
                label, state, position);
        Stream.of(c_flaw.causes).forEach(c -> c.preconditions.add(c_flaw));
        flaws.put(id, c_flaw);
        try {
            App.broadcast(App.MAPPER.writeValueAsString(c_flaw));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void flawStateChanged(final long id, final State state) {
        final Flaw flaw = flaws.get(id);
        flaw.state = state;
        try {
            App.broadcast(App.MAPPER.writeValueAsString(flaw));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void flawCostChanged(final long id, final Rational cost) {
        final Flaw flaw = flaws.get(id);
        flaw.cost = cost;
        try {
            App.broadcast(App.MAPPER.writeValueAsString(flaw));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void flawPositionChanged(final long id, final Bound position) {
        final Flaw flaw = flaws.get(id);
        flaw.position = position;
        try {
            App.broadcast(App.MAPPER.writeValueAsString(flaw));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void currentFlaw(final long id) {
        if (current_flaw != null)
            current_flaw.current = false;
        final Flaw flaw = flaws.get(id);
        current_flaw = flaw;
        current_flaw.current = true;
        try {
            App.broadcast(App.MAPPER.writeValueAsString(flaw));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void resolverCreated(final long id, final long effect, final Rational cost, final String label,
            final State state) {
        final Resolver resolver = new Resolver(id, flaws.get(effect), label, state, cost);
        resolvers.put(id, resolver);
        try {
            App.broadcast(App.MAPPER.writeValueAsString(resolver));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void resolverStateChanged(final long id, final State state) {
        final Resolver resolver = resolvers.get(id);
        resolver.state = state;
        try {
            App.broadcast(App.MAPPER.writeValueAsString(resolver));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void currentResolver(final long id) {
        if (current_resolver != null)
            current_resolver.current = false;
        final Resolver resolver = resolvers.get(id);
        current_resolver = resolver;
        current_resolver.current = true;
        try {
            App.broadcast(App.MAPPER.writeValueAsString(resolver));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void causalLinkAdded(final long flaw, final long resolver) {
        resolvers.get(resolver).preconditions.add(flaws.get(flaw));
        try {
            App.broadcast(App.MAPPER.writeValueAsString(null));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
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

        private Flaw(long id, Resolver[] causes, String label, State state, Bound position) {
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
        public void serialize(Flaw flaw, JsonGenerator gen, SerializerProvider provider) throws IOException {
            gen.writeStartObject();
            gen.writeNumberField("id", flaw.id);
            gen.writeArrayFieldStart("causes");
            Arrays.stream(flaw.causes).forEach(c -> {
                try {
                    gen.writeNumber(c.id);
                } catch (IOException e) {
                    LOG.error("Cannot serialize", e);
                }
            });
            gen.writeEndArray();
            gen.writeStringField("label", flaw.label);
            gen.writeNumberField("state", flaw.state.ordinal());

            if (flaw.position.min != -Bound.INF || flaw.position.max != Bound.INF) {
                gen.writeFieldName("position");
                gen.writeStartObject();
                if (flaw.position.min != -Bound.INF)
                    gen.writeNumberField("min", flaw.position.min);
                if (flaw.position.max != Bound.INF)
                    gen.writeNumberField("max", flaw.position.max);
                gen.writeEndObject();
            }

            gen.writeFieldName("cost");
            gen.writeStartObject();
            gen.writeNumberField("num", flaw.cost.getNumerator());
            gen.writeNumberField("den", flaw.cost.getDenominator());
            gen.writeEndObject();

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

        private Resolver(long id, Flaw effect, String label, State state, Rational cost) {
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
        public void serialize(Resolver resolver, JsonGenerator gen, SerializerProvider provider) throws IOException {
            gen.writeStartObject();
            gen.writeNumberField("id", resolver.id);
            gen.writeNumberField("effect", resolver.effect.id);
            gen.writeStringField("label", resolver.label);
            gen.writeNumberField("state", resolver.state.ordinal());

            gen.writeFieldName("cost");
            gen.writeStartObject();
            gen.writeNumberField("num", resolver.cost.getNumerator());
            gen.writeNumberField("den", resolver.cost.getDenominator());
            gen.writeEndObject();
            gen.writeArrayFieldStart("preconditions");
            resolver.preconditions.stream().forEach(pre -> {
                try {
                    gen.writeNumber(pre.id);
                } catch (IOException e) {
                    LOG.error("Cannot serialize", e);
                }
            });
            gen.writeEndArray();

            gen.writeBooleanField("current", resolver.current);

            gen.writeEndObject();
        }
    }
}
