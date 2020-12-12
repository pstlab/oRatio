package it.cnr.istc.pst.oratio.gui;

import java.io.IOException;
import java.util.Collection;
import java.util.HashSet;
import java.util.Properties;
import java.util.Set;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.fasterxml.jackson.annotation.JsonSubTypes.Type;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.javalin.Javalin;
import io.javalin.websocket.WsContext;
import it.cnr.istc.pst.oratio.Context;

public class App {

    private static final Logger LOG = LoggerFactory.getLogger(App.class);
    static final Executor EXECUTOR = Executors.newSingleThreadExecutor();
    static final ObjectMapper MAPPER = new ObjectMapper();
    static final SolverState STATE = new SolverState();
    static final CausalGraph GRAPH = new CausalGraph();
    private static Set<WsContext> contexts = new HashSet<>();

    public static void main(final String[] args) {
        Properties properties = new Properties();
        try {
            properties.load(App.class.getResourceAsStream("/config.properties"));
        } catch (IOException e) {
            LOG.error("Cannot load properties", e);
        }

        Context.getContext().addStateListener(STATE);
        Context.getContext().addGraphListener(GRAPH);

        new Thread(
                () -> Context.getContext().startServer(Integer.parseInt(properties.getProperty("server.port", "1100"))))
                        .start();

        final Javalin app = Javalin.create(config -> {
            config.addStaticFiles("/public");
        });
        app.ws("/graph", ws -> {
            ws.onConnect(ctx -> {
                EXECUTOR.execute(() -> {
                    LOG.info("New connection..");
                    contexts.add(ctx);
                    try {
                        broadcast(MAPPER.writeValueAsString(new Message.Timelines(STATE.getTimelines())));
                        broadcast(MAPPER.writeValueAsString(new Message.Graph(GRAPH)));
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
            @Type(value = Message.Timelines.class, name = "timelines") })
    public static abstract class Message {

        public static class Log extends Message {

            public final String log;

            public Log(final String log) {
                this.log = log;
            }
        }

        public static class Graph extends Message {

            public final CausalGraph graph;

            public Graph(final CausalGraph graph) {
                this.graph = graph;
            }
        }

        public static class Timelines extends Message {

            public final Collection<Object> timelines;

            public Timelines(final Collection<Object> timelines) {
                this.timelines = timelines;
            }
        }
    }
}
