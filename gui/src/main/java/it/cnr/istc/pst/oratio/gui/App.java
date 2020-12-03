package it.cnr.istc.pst.oratio.gui;

import java.io.IOException;
import java.util.HashSet;
import java.util.Properties;
import java.util.Set;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.javalin.Javalin;
import io.javalin.websocket.WsContext;
import it.cnr.istc.pst.oratio.Context;

public class App {

    static final Logger LOG = LoggerFactory.getLogger(App.class);
    static final Executor EXECUTOR = Executors.newSingleThreadExecutor();
    static final Gson GSON = new GsonBuilder().registerTypeAdapter(CausalGraph.class, new CausalGraph.GraphSerializer())
            .registerTypeAdapter(CausalGraph.Flaw.class, new CausalGraph.FlawSerializer())
            .registerTypeAdapter(CausalGraph.Resolver.class, new CausalGraph.ResolverSerializer()).create();
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
                    broadcast("timelines " + App.GSON.toJson(STATE.getTimelines()));
                    broadcast("graph " + GSON.toJson(GRAPH));
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
}
