package it.cnr.istc.pst.oratio.gui;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonSubTypes.Type;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.javalin.Javalin;
import io.javalin.websocket.WsContext;
import it.cnr.istc.pst.oratio.Bound;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.gui.CausalGraph.Flaw;
import it.cnr.istc.pst.oratio.gui.CausalGraph.Resolver;

public class App {

    private static final Logger LOG = LoggerFactory.getLogger(App.class);
    static final Executor EXECUTOR = Executors.newSingleThreadExecutor();
    static final ObjectMapper MAPPER = new ObjectMapper();
    static final Solver SOLVER = new Solver();
    static final SolverState STATE = new SolverState(SOLVER);
    static final CausalGraph GRAPH = new CausalGraph(SOLVER);
    static final PlanExecutor PLAN_EXEC = new PlanExecutor(SOLVER);
    private static Set<WsContext> contexts = new HashSet<>();

    public static void main(final String[] args) {
        SOLVER.addStateListener(STATE);
        SOLVER.addGraphListener(GRAPH);
        SOLVER.addExecutorListener(PLAN_EXEC);

        final Javalin app = Javalin.create(config -> {
            config.addStaticFiles("/public");
        });
        app.ws("/graph", ws -> {
            ws.onConnect(ctx -> {
                EXECUTOR.execute(() -> {
                    LOG.info("New connection..");
                    contexts.add(ctx);
                    try {
                        broadcast(MAPPER.writeValueAsString(new Message.Graph(GRAPH)));
                        broadcast(MAPPER.writeValueAsString(new Message.Timelines(STATE.getTimelines())));
                        broadcast(MAPPER.writeValueAsString(new Message.Tick(PLAN_EXEC.getCurrentTime())));
                    } catch (JsonProcessingException e) {
                        LOG.error("Cannot serialize", e);
                    }
                });
            });
            ws.onClose(ctx -> {
                EXECUTOR.execute(() -> {
                    LOG.info("Lost connection..");
                    contexts.remove(ctx);
                });
            });
            ws.onMessage(ctx -> EXECUTOR.execute(() -> LOG.info("Received message {}..", ctx)));
        });
        app.start();
    }

    static void broadcast(final String message) {
        contexts.forEach(session -> session.send(message));
    }

    @SuppressWarnings({ "unused" })
    @JsonTypeInfo(use = JsonTypeInfo.Id.NAME, include = JsonTypeInfo.As.PROPERTY, property = "message_type")
    @JsonSubTypes({ @Type(value = Message.Log.class, name = "log"), @Type(value = Message.Graph.class, name = "graph"),
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

            final String log;

            Log(final String log) {
                this.log = log;
            }
        }

        static class Graph extends Message {

            final Collection<Flaw> flaws;
            final Flaw current_flaw;
            final Collection<Resolver> resolvers;
            final Resolver current_resolver;

            Graph(final CausalGraph graph) {
                this.flaws = graph.getFlaws();
                this.current_flaw = graph.getCurrentFlaw();
                this.resolvers = graph.getResolvers();
                this.current_resolver = graph.getCurrentResolver();
            }
        }

        static class FlawCreated extends Message {

            final long id;
            final long[] causes;
            final String label;
            final byte state;
            final Bound position;

            FlawCreated(long id, long[] causes, String label, byte state, Bound position) {
                this.id = id;
                this.causes = causes;
                this.label = label;
                this.state = state;
                this.position = position;
            }
        }

        static class FlawStateChanged extends Message {

            final long id;
            final byte state;

            FlawStateChanged(long id, byte state) {
                this.id = id;
                this.state = state;
            }
        }

        static class FlawCostChanged extends Message {

            final long id;
            final Rational cost;

            FlawCostChanged(long id, Rational cost) {
                this.id = id;
                this.cost = cost;
            }
        }

        static class FlawPositionChanged extends Message {

            final long id;
            final Bound position;

            FlawPositionChanged(long id, Bound position) {
                this.id = id;
                this.position = position;
            }
        }

        static class CurrentFlaw extends Message {

            final long id;

            CurrentFlaw(long id) {
                this.id = id;
            }
        }

        static class ResolverCreated extends Message {

            final long id;
            final long effect;
            final Rational cost;
            final long label;
            final byte state;

            ResolverCreated(long id, long effect, Rational cost, long label, byte state) {
                this.id = id;
                this.effect = effect;
                this.cost = cost;
                this.label = label;
                this.state = state;
            }
        }

        static class ResolverStateChanged extends Message {

            final long id;
            final byte state;

            ResolverStateChanged(long id, byte state) {
                this.id = id;
                this.state = state;
            }
        }

        static class CurrentResolver extends Message {

            final long id;

            CurrentResolver(long id) {
                this.id = id;
            }
        }

        static class CausalLinkAdded extends Message {

            final long flaw;
            final long resolver;

            CausalLinkAdded(long flaw, long resolver) {
                this.flaw = flaw;
                this.resolver = resolver;
            }
        }

        static class Timelines extends Message {

            final Collection<Object> timelines;

            Timelines(final Collection<Object> timelines) {
                this.timelines = timelines;
            }
        }

        static class Tick extends Message {

            final Rational current_time;

            Tick(Rational current_time) {
                this.current_time = current_time;
            }
        }

        static class StartingAtoms extends Message {

            final long[] atoms;

            StartingAtoms(long[] atoms) {
                this.atoms = atoms;
            }
        }

        static class EndingAtoms extends Message {

            final long[] atoms;

            EndingAtoms(long[] atoms) {
                this.atoms = atoms;
            }
        }
    }
}
