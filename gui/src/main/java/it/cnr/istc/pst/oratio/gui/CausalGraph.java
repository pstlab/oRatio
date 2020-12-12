package it.cnr.istc.pst.oratio.gui;

import java.io.IOException;
import java.util.Arrays;
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

import it.cnr.istc.pst.oratio.Context;
import it.cnr.istc.pst.oratio.Context.Bound;
import it.cnr.istc.pst.oratio.Context.Message.CausalLinkAdded;
import it.cnr.istc.pst.oratio.Context.Message.CurrentFlaw;
import it.cnr.istc.pst.oratio.Context.Message.CurrentResolver;
import it.cnr.istc.pst.oratio.Context.Message.FlawCostChanged;
import it.cnr.istc.pst.oratio.Context.Message.FlawCreated;
import it.cnr.istc.pst.oratio.Context.Message.FlawPositionChanged;
import it.cnr.istc.pst.oratio.Context.Message.FlawStateChanged;
import it.cnr.istc.pst.oratio.Context.Message.ResolverCreated;
import it.cnr.istc.pst.oratio.Context.Message.ResolverStateChanged;
import it.cnr.istc.pst.oratio.GraphListener;
import it.cnr.istc.pst.oratio.gui.App.Message;
import it.cnr.istc.pst.oratio.riddle.Rational;

@JsonSerialize(using = CausalGraph.GraphSerializer.class)
public class CausalGraph implements GraphListener {

    private static final Logger LOG = LoggerFactory.getLogger(CausalGraph.class);
    private final Map<String, Flaw> flaws = new HashMap<>();
    private Flaw current_flaw = null;
    private final Map<String, Resolver> resolvers = new HashMap<>();
    private Resolver current_resolver = null;

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

    @Override
    public void flawCreated(final FlawCreated flaw) {
        App.EXECUTOR.execute(() -> {
            Flaw c_flaw = new Flaw(flaw.id,
                    Stream.of(flaw.causes).map(id -> resolvers.get(id)).toArray(Resolver[]::new), flaw.label,
                    flaw.state, flaw.position);
            Stream.of(c_flaw.causes).forEach(c -> c.preconditions.add(c_flaw));
            flaws.put(flaw.id, c_flaw);
            try {
                App.broadcast(App.MAPPER.writeValueAsString(flaw));
            } catch (JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void flawStateChanged(final FlawStateChanged flaw) {
        App.EXECUTOR.execute(() -> {
            flaws.get(flaw.id).state = flaw.state;
            try {
                App.broadcast(App.MAPPER.writeValueAsString(flaw));
            } catch (JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void flawCostChanged(final FlawCostChanged flaw) {
        App.EXECUTOR.execute(() -> {
            flaws.get(flaw.id).cost = flaw.cost;
            try {
                App.broadcast(App.MAPPER.writeValueAsString(flaw));
            } catch (JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void flawPositionChanged(FlawPositionChanged fpc) {
        App.EXECUTOR.execute(() -> {
            flaws.get(fpc.id).position = fpc.position;
            try {
                App.broadcast(App.MAPPER.writeValueAsString(fpc));
            } catch (JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void currentFlaw(final CurrentFlaw flaw) {
        App.EXECUTOR.execute(() -> {
            if (current_flaw != null)
                current_flaw.current = false;
            current_flaw = flaws.get(flaw.id);
            current_flaw.current = true;
            try {
                App.broadcast(App.MAPPER.writeValueAsString(flaw));
            } catch (JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void resolverCreated(final ResolverCreated resolver) {
        App.EXECUTOR.execute(() -> {
            resolvers.put(resolver.id, new Resolver(resolver.id, flaws.get(resolver.effect), resolver.label,
                    resolver.state, resolver.cost));
            try {
                App.broadcast(App.MAPPER.writeValueAsString(resolver));
            } catch (JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void resolverStateChanged(final ResolverStateChanged resolver) {
        App.EXECUTOR.execute(() -> {
            resolvers.get(resolver.id).state = resolver.state;
            try {
                App.broadcast(App.MAPPER.writeValueAsString(resolver));
            } catch (JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void currentResolver(final CurrentResolver resolver) {
        App.EXECUTOR.execute(() -> {
            if (current_resolver != null)
                current_resolver.current = false;
            current_resolver = resolvers.get(resolver.id);
            current_resolver.current = true;
            try {
                App.broadcast(App.MAPPER.writeValueAsString(resolver));
            } catch (JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void causalLinkAdded(final CausalLinkAdded causal_link) {
        App.EXECUTOR.execute(() -> {
            resolvers.get(causal_link.resolver).preconditions.add(flaws.get(causal_link.flaw));
            try {
                App.broadcast(App.MAPPER.writeValueAsString(causal_link));
            } catch (JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @JsonSerialize(using = FlawSerializer.class)
    class Flaw {

        private final String id;
        private final Resolver[] causes;
        private final String label;
        private int state;
        private Bound position;
        private Rational cost = Rational.POSITIVE_INFINITY;
        private boolean current = false;

        private Flaw(String id, Resolver[] causes, String label, int state, Bound position) {
            this.id = id;
            this.causes = causes;
            this.label = label;
            this.state = state;
            this.position = position;
        }
    }

    static class FlawSerializer extends StdSerializer<Flaw> {

        private static final long serialVersionUID = 1L;

        private FlawSerializer() {
            super(Flaw.class);
        }

        @Override
        public void serialize(Flaw flaw, JsonGenerator gen, SerializerProvider provider) throws IOException {
            gen.writeStartObject();
            gen.writeStringField("id", flaw.id);
            gen.writeArrayFieldStart("causes");
            Arrays.stream(flaw.causes).forEach(c -> {
                try {
                    gen.writeString(c.id);
                } catch (IOException e) {
                    LOG.error("Cannot serialize", e);
                }
            });
            gen.writeEndArray();
            gen.writeStringField("label", flaw.label);
            gen.writeNumberField("state", flaw.state);

            if (flaw.position.min != -Context.INF || flaw.position.max != Context.INF) {
                gen.writeFieldName("position");
                gen.writeStartObject();
                if (flaw.position.min != -Context.INF)
                    gen.writeNumberField("min", flaw.position.min);
                if (flaw.position.max != Context.INF)
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

        private final String id;
        private final Flaw effect;
        private final String label;
        private int state;
        private final Rational cost;
        private final Set<Flaw> preconditions = new HashSet<>();
        private boolean current = false;

        private Resolver(String id, Flaw effect, String label, int state, Rational cost) {
            this.id = id;
            this.effect = effect;
            this.label = label;
            this.state = state;
            this.cost = cost;
        }
    }

    static class ResolverSerializer extends StdSerializer<Resolver> {

        protected ResolverSerializer() {
            super(Resolver.class);
        }

        /**
         *
         */
        private static final long serialVersionUID = 1L;

        @Override
        public void serialize(Resolver resolver, JsonGenerator gen, SerializerProvider provider) throws IOException {
            gen.writeStartObject();
            gen.writeStringField("id", resolver.id);
            gen.writeStringField("effect", resolver.effect.id);
            gen.writeStringField("label", resolver.label);
            gen.writeNumberField("state", resolver.state);

            gen.writeFieldName("cost");
            gen.writeStartObject();
            gen.writeNumberField("num", resolver.cost.getNumerator());
            gen.writeNumberField("den", resolver.cost.getDenominator());
            gen.writeEndObject();
            gen.writeArrayFieldStart("preconditions");
            resolver.preconditions.stream().forEach(pre -> {
                try {
                    gen.writeString(pre.id);
                } catch (IOException e) {
                    LOG.error("Cannot serialize", e);
                }
            });
            gen.writeEndArray();

            gen.writeBooleanField("current", resolver.current);

            gen.writeEndObject();
        }
    }

    static class GraphSerializer extends StdSerializer<CausalGraph> {

        private static final long serialVersionUID = 1L;

        private GraphSerializer() {
            super(CausalGraph.class);
        }

        @Override
        public void serialize(CausalGraph graph, JsonGenerator gen, SerializerProvider provider) throws IOException {
            gen.writeStartObject();
            gen.writeArrayFieldStart("flaws");
            graph.flaws.values().stream().forEach(f -> {
                try {
                    gen.writeObject(f);
                } catch (IOException e) {
                    LOG.error("Cannot serialize", e);
                }
            });
            gen.writeEndArray();
            if (graph.current_flaw != null)
                gen.writeStringField("current_flaw", graph.current_flaw.id);
            gen.writeArrayFieldStart("resolvers");
            graph.resolvers.values().stream().forEach(r -> {
                try {
                    gen.writeObject(r);
                } catch (IOException e) {
                    LOG.error("Cannot serialize", e);
                }
            });
            gen.writeEndArray();
            if (graph.current_resolver != null)
                gen.writeStringField("current_resolver", graph.current_resolver.id);
            gen.writeEndObject();
        }
    }
}
