package it.cnr.istc.pst.oratio.gui;

import java.util.HashSet;
import java.util.Set;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.javalin.Javalin;
import io.javalin.websocket.WsContext;
import io.javalin.websocket.WsMessageContext;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.SolverException;
import it.cnr.istc.pst.oratio.gui.SolverListener.SolverState;
import it.cnr.istc.pst.oratio.timelines.ExecutorException;
import it.cnr.istc.pst.oratio.timelines.TimelinesExecutor;
import it.cnr.istc.pst.oratio.utils.Message;

public class App {

    private static final Logger LOG = LoggerFactory.getLogger(App.class);
    static final ObjectMapper MAPPER = new ObjectMapper();
    static final Solver SOLVER = new Solver();
    static final TimelinesExecutor TL_EXEC = new TimelinesExecutor(SOLVER, "{}", new Rational(1));
    static final SolverListener SLV_LISTENER = new SolverListener(SOLVER, TL_EXEC);
    static {
        SOLVER.addStateListener(SLV_LISTENER);
        SOLVER.addGraphListener(SLV_LISTENER);
        TL_EXEC.addExecutorListener(SLV_LISTENER);
    }
    private static Set<WsContext> contexts = new HashSet<>();

    public static void main(final String[] args) {
        start_server();

        try {
            SOLVER.read(args);
            SOLVER.solve();
        } catch (SolverException e) {
            LOG.error("Cannot solve the problem", e);
        }
    }

    public static void start_server() {
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
            ctx.send(MAPPER.writeValueAsString(new Message.Timelines(SLV_LISTENER.getSolver().getTimelines())));
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
                try {
                    TL_EXEC.tick();
                } catch (ExecutorException e) {
                    LOG.error("Cannot execute the solution", e);
                }
                break;
            default:
                break;
        }
    }

    static void broadcast(final String message) {
        contexts.forEach(session -> session.send(message));
    }
}
