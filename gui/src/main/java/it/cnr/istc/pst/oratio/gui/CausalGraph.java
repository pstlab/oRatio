package it.cnr.istc.pst.oratio.gui;

import java.lang.reflect.Type;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.stream.Stream;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonSerializationContext;
import com.google.gson.JsonSerializer;

import it.cnr.istc.pst.oratio.GraphListener;
import it.cnr.istc.pst.oratio.riddle.Rational;

public class CausalGraph implements GraphListener {

    private static final String GRAPH = "graph ";
    private static final String FLAW_CREATED = "flaw_created ";
    private static final String FLAW_STATE_CHANGED = "flaw_state_changed ";
    private static final String FLAW_COST_CHANGED = "flaw_cost_changed ";
    private static final String FLAW_POSITION_CHANGED = "flaw_position_changed ";
    private static final String CURRENT_FLAW = "current_flaw ";
    private static final String RESOLVER_CREATED = "resolver_created ";
    private static final String RESOLVER_STATE_CHANGED = "resolver_state_changed ";
    private static final String CURRENT_RESOLVER = "current_resolver ";
    private static final String CAUSAL_LINK_ADDED = "causal_link_added ";

    private final Map<String, Flaw> flaws = new HashMap<>();
    private Flaw current_flaw = null;
    private final Map<String, Resolver> resolvers = new HashMap<>();
    private Resolver current_resolver = null;

    void clear() {
        flaws.clear();
        resolvers.clear();
        current_flaw = null;
        current_resolver = null;
        App.broadcast(GRAPH + App.GSON.toJson(this));
    }

    @Override
    public void flawCreated(final FlawCreated flaw) {
        App.EXECUTOR.execute(() -> {
            Flaw c_flaw = new Flaw(flaw.id,
                    Stream.of(flaw.causes).map(id -> resolvers.get(id)).toArray(Resolver[]::new), flaw.label,
                    flaw.state, flaw.position);
            Stream.of(c_flaw.causes).forEach(c -> c.preconditions.add(c_flaw));
            flaws.put(flaw.id, c_flaw);
            App.broadcast(FLAW_CREATED + App.GSON.toJson(flaw));
        });
    }

    @Override
    public void flawStateChanged(final FlawStateChanged flaw) {
        App.EXECUTOR.execute(() -> {
            flaws.get(flaw.id).state = flaw.state;
            App.broadcast(FLAW_STATE_CHANGED + App.GSON.toJson(flaw));
        });
    }

    @Override
    public void flawCostChanged(final FlawCostChanged flaw) {
        App.EXECUTOR.execute(() -> {
            flaws.get(flaw.id).cost = flaw.cost;
            App.broadcast(FLAW_COST_CHANGED + App.GSON.toJson(flaw));
        });
    }

    @Override
    public void flawPositionChanged(FlawPositionChanged fpc) {
        App.EXECUTOR.execute(() -> {
            flaws.get(fpc.id).position = fpc.position;
            App.broadcast(FLAW_POSITION_CHANGED + App.GSON.toJson(fpc));
        });
    }

    @Override
    public void currentFlaw(final CurrentFlaw flaw) {
        App.EXECUTOR.execute(() -> {
            current_flaw = flaws.get(flaw.id);
            App.broadcast(CURRENT_FLAW + App.GSON.toJson(flaw));
        });
    }

    @Override
    public void resolverCreated(final ResolverCreated resolver) {
        App.EXECUTOR.execute(() -> {
            resolvers.put(resolver.id, new Resolver(resolver.id, flaws.get(resolver.effect), resolver.label,
                    resolver.state, resolver.cost));
            App.broadcast(RESOLVER_CREATED + App.GSON.toJson(resolver));
        });
    }

    @Override
    public void resolverStateChanged(final ResolverStateChanged resolver) {
        App.EXECUTOR.execute(() -> {
            resolvers.get(resolver.id).state = resolver.state;
            App.broadcast(RESOLVER_STATE_CHANGED + App.GSON.toJson(resolver));
        });
    }

    @Override
    public void currentResolver(final CurrentResolver resolver) {
        App.EXECUTOR.execute(() -> {
            current_resolver = resolvers.get(resolver.id);
            App.broadcast(CURRENT_RESOLVER + App.GSON.toJson(resolver));
        });
    }

    @Override
    public void causalLinkAdded(final CausalLinkAdded causal_link) {
        App.EXECUTOR.execute(() -> {
            resolvers.get(causal_link.resolver_id).preconditions.add(flaws.get(causal_link.flaw_id));
            App.broadcast(CAUSAL_LINK_ADDED + App.GSON.toJson(causal_link));
        });
    }

    class Flaw {

        private final String id;
        private final Resolver[] causes;
        private final String label;
        private int state;
        private Bound position;
        private Rational cost = new Rational();

        private Flaw(String id, Resolver[] causes, String label, int state, Bound position) {
            this.id = id;
            this.causes = causes;
            this.label = label;
            this.state = state;
            this.position = position;
        }
    }

    static class FlawSerializer implements JsonSerializer<Flaw> {

        @Override
        public JsonElement serialize(Flaw src, Type typeOfSrc, JsonSerializationContext context) {
            JsonObject flaw = new JsonObject();
            flaw.addProperty("id", src.id);
            JsonArray causes = new JsonArray();
            Arrays.stream(src.causes).forEach(c -> causes.add(c.id));
            flaw.add("causes", causes);
            flaw.addProperty("label", src.label);
            flaw.addProperty("state", src.state);
            JsonObject position = new JsonObject();
            if (src.position.min != -GraphListener.INF)
                position.addProperty("min", src.position.min);
            if (src.position.max != GraphListener.INF)
                position.addProperty("max", src.position.max);
            if (position.has("min") | position.has("max"))
                flaw.add("position", position);
            JsonObject cost = new JsonObject();
            cost.addProperty("num", src.cost.getNumerator());
            cost.addProperty("den", src.cost.getDenominator());
            flaw.add("cost", cost);
            return flaw;
        }
    }

    class Resolver {

        private final String id;
        private final Flaw effect;
        private final String label;
        private int state;
        private final Rational cost;
        private final Set<Flaw> preconditions = new HashSet<>();

        private Resolver(String id, Flaw effect, String label, int state, Rational cost) {
            this.id = id;
            this.effect = effect;
            this.label = label;
            this.state = state;
            this.cost = cost;
        }
    }

    static class ResolverSerializer implements JsonSerializer<Resolver> {

        @Override
        public JsonElement serialize(Resolver src, Type typeOfSrc, JsonSerializationContext context) {
            JsonObject resolver = new JsonObject();
            resolver.addProperty("id", src.id);
            resolver.addProperty("effect", src.effect.id);
            resolver.addProperty("label", src.label);
            resolver.addProperty("state", src.state);
            JsonObject cost = new JsonObject();
            cost.addProperty("num", src.cost.getNumerator());
            cost.addProperty("den", src.cost.getDenominator());
            resolver.add("cost", cost);
            JsonArray preconditions = new JsonArray();
            src.preconditions.stream().forEach(c -> preconditions.add(c.id));
            resolver.add("preconditions", preconditions);
            return resolver;
        }
    }

    static class GraphSerializer implements JsonSerializer<CausalGraph> {

        @Override
        public JsonElement serialize(CausalGraph src, Type typeOfSrc, JsonSerializationContext context) {
            JsonObject graph = new JsonObject();
            JsonArray flaws = new JsonArray();
            src.flaws.values().stream().forEach(c -> flaws.add(App.GSON.toJson(c)));
            graph.add("flaws", flaws);
            if (src.current_flaw != null)
                graph.add("current_flaw", App.GSON.toJsonTree(src.current_flaw));
            JsonArray resolvers = new JsonArray();
            src.resolvers.values().stream().forEach(c -> resolvers.add(App.GSON.toJson(c)));
            graph.add("resolvers", resolvers);
            if (src.current_resolver != null)
                graph.add("current_resolver", App.GSON.toJsonTree(src.current_resolver));
            return graph;
        }
    }
}
