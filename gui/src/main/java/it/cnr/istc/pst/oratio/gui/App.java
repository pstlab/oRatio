package it.cnr.istc.pst.oratio.gui;

import java.io.IOException;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonSubTypes.Type;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.module.SimpleModule;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.javalin.Javalin;
import io.javalin.websocket.WsContext;
import io.javalin.websocket.WsMessageContext;
import it.cnr.istc.pst.oratio.Bound;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.gui.SolverListener.Flaw;
import it.cnr.istc.pst.oratio.gui.SolverListener.Resolver;
import it.cnr.istc.pst.oratio.gui.SolverListener.SolverState;
import it.cnr.istc.pst.oratio.timelines.TimelinesExecutor;

public class App {

    private static final Logger LOG = LoggerFactory.getLogger(App.class);
    static final ObjectMapper MAPPER = new ObjectMapper();
    static final Solver SOLVER = new Solver();
    static final TimelinesExecutor TL_EXEC = new TimelinesExecutor(SOLVER, new Rational(1));
    static final SolverListener SLV_LISTENER = new SolverListener(SOLVER, TL_EXEC);
    static {
        SOLVER.addStateListener(SLV_LISTENER);
        SOLVER.addGraphListener(SLV_LISTENER);
        TL_EXEC.addExecutorListener(SLV_LISTENER);
    }
    private static Set<WsContext> contexts = new HashSet<>();

    public static void main(final String[] args) {
        start_server();

        SOLVER.read(args);
        SOLVER.solve();
    }

    public static void start_server() {
        final SimpleModule module = new SimpleModule();
        module.addSerializer(Rational.class, new StdSerializer<Rational>(Rational.class) {

            private static final long serialVersionUID = 1L;

            @Override
            public void serialize(final Rational value, final JsonGenerator gen, final SerializerProvider serializers)
                    throws IOException {
                gen.writeStartObject();
                gen.writeNumberField("num", value.getNumerator());
                gen.writeNumberField("den", value.getDenominator());
                gen.writeEndObject();
            }
        });
        module.addSerializer(Bound.class, new StdSerializer<Bound>(Bound.class) {

            private static final long serialVersionUID = 1L;

            @Override
            public void serialize(final Bound value, final JsonGenerator gen, final SerializerProvider provider)
                    throws IOException {
                gen.writeStartObject();
                if (value.min != -Bound.INF)
                    gen.writeNumberField("min", value.min);
                if (value.max != Bound.INF)
                    gen.writeNumberField("max", value.max);
                gen.writeEndObject();
            }
        });
        MAPPER.registerModule(module);

        final Javalin app = Javalin.create(config -> {
            config.addStaticFiles("/public");
        });
        app.ws("/solver", ws -> {
            ws.onConnect(ctx -> on_connect(ctx));
            ws.onClose(ctx -> on_close(ctx));
            ws.onMessage(ctx -> on_message(ctx));
        });
        app.start();
    }

    static synchronized void on_connect(final WsContext ctx) {
        LOG.info("New connection..");
        contexts.add(ctx);
        try {
            ctx.send(
                    MAPPER.writeValueAsString(new Message.Graph(SLV_LISTENER.getFlaws(), SLV_LISTENER.getResolvers())));
            ctx.send(MAPPER.writeValueAsString(new Message.Timelines(SLV_LISTENER.getTimelines())));
            if (SLV_LISTENER.getState() == SolverState.Solved)
                ctx.send(MAPPER.writeValueAsString(new Message.Tick(SLV_LISTENER.getCurrentTime())));
        } catch (final JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    static synchronized void on_close(final WsContext ctx) {
        LOG.info("Lost connection..");
        contexts.remove(ctx);
    }

    static synchronized void on_message(final WsMessageContext ctx) {
        final String message = ctx.message();
        LOG.info("Received message {}..", message);
        switch (message) {
        case "tick":
            TL_EXEC.tick();
            break;
        default:
            break;
        }
    }

    static void broadcast(final String message) {
        contexts.forEach(session -> session.send(message));
    }

    @SuppressWarnings({ "unused" })
    @JsonTypeInfo(use = JsonTypeInfo.Id.NAME, include = JsonTypeInfo.As.PROPERTY, property = "message_type")
    @JsonSubTypes({ @Type(value = Message.Log.class, name = "log"),
            @Type(value = Message.StartedSolving.class, name = "started_solving"),
            @Type(value = Message.SolutionFound.class, name = "solution_found"),
            @Type(value = Message.InconsistentProblem.class, name = "inconsistent_problem"),
            @Type(value = Message.Graph.class, name = "graph"),
            @Type(value = Message.FlawCreated.class, name = "flaw_created"),
            @Type(value = Message.FlawStateChanged.class, name = "flaw_state_changed"),
            @Type(value = Message.FlawCostChanged.class, name = "flaw_cost_changed"),
            @Type(value = Message.FlawPositionChanged.class, name = "flaw_position_changed"),
            @Type(value = Message.CurrentFlaw.class, name = "current_flaw"),
            @Type(value = Message.ResolverCreated.class, name = "resolver_created"),
            @Type(value = Message.ResolverStateChanged.class, name = "resolver_state_changed"),
            @Type(value = Message.CurrentResolver.class, name = "current_resolver"),
            @Type(value = Message.CausalLinkAdded.class, name = "causal_link_added"),
            @Type(value = Message.Timelines.class, name = "timelines"),
            @Type(value = Message.Tick.class, name = "tick"),
            @Type(value = Message.StartingAtoms.class, name = "starting_atoms"),
            @Type(value = Message.EndingAtoms.class, name = "ending_atoms") })
    static abstract class Message {

        static class Log extends Message {

            public final String log;

            Log(final String log) {
                this.log = log;
            }
        }

        static class StartedSolving extends Message {
        }

        static class SolutionFound extends Message {
        }

        static class InconsistentProblem extends Message {
        }

        static class Graph extends Message {

            public final Collection<Flaw> flaws;
            public final Collection<Resolver> resolvers;

            Graph(final Collection<Flaw> flaws, final Collection<Resolver> resolvers) {
                this.flaws = flaws;
                this.resolvers = resolvers;
            }
        }

        static class FlawCreated extends Message {

            public final long id;
            public final long[] causes;
            public final String label;
            public final byte state;
            public final Bound position;

            FlawCreated(final long id, final long[] causes, final String label, final byte state,
                    final Bound position) {
                this.id = id;
                this.causes = causes;
                this.label = label;
                this.state = state;
                this.position = position;
            }
        }

        static class FlawStateChanged extends Message {

            public final long id;
            public final byte state;

            FlawStateChanged(final long id, final byte state) {
                this.id = id;
                this.state = state;
            }
        }

        static class FlawCostChanged extends Message {

            public final long id;
            public final Rational cost;

            FlawCostChanged(final long id, final Rational cost) {
                this.id = id;
                this.cost = cost;
            }
        }

        static class FlawPositionChanged extends Message {

            public final long id;
            public final Bound position;

            FlawPositionChanged(final long id, final Bound position) {
                this.id = id;
                this.position = position;
            }
        }

        static class CurrentFlaw extends Message {

            public final long id;

            CurrentFlaw(final long id) {
                this.id = id;
            }
        }

        static class ResolverCreated extends Message {

            public final long id;
            public final long effect;
            public final Rational cost;
            public final String label;
            public final byte state;

            ResolverCreated(final long id, final long effect, final Rational cost, final String label,
                    final byte state) {
                this.id = id;
                this.effect = effect;
                this.cost = cost;
                this.label = label;
                this.state = state;
            }
        }

        static class ResolverStateChanged extends Message {

            public final long id;
            public final byte state;

            ResolverStateChanged(final long id, final byte state) {
                this.id = id;
                this.state = state;
            }
        }

        static class CurrentResolver extends Message {

            public final long id;

            CurrentResolver(final long id) {
                this.id = id;
            }
        }

        static class CausalLinkAdded extends Message {

            public final long flaw;
            public final long resolver;

            CausalLinkAdded(final long flaw, final long resolver) {
                this.flaw = flaw;
                this.resolver = resolver;
            }
        }

        static class Timelines extends Message {

            public final Collection<Object> timelines;

            Timelines(final Collection<Object> timelines) {
                this.timelines = timelines;
            }
        }

        static class Tick extends Message {

            public final Rational current_time;

            Tick(final Rational current_time) {
                this.current_time = current_time;
            }
        }

        static class StartingAtoms extends Message {

            public final long[] atoms;

            StartingAtoms(final long[] atoms) {
                this.atoms = atoms;
            }
        }

        static class EndingAtoms extends Message {

            public final long[] atoms;

            EndingAtoms(final long[] atoms) {
                this.atoms = atoms;
            }
        }
    }
}
