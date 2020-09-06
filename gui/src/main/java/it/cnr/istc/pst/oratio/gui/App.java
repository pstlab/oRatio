package it.cnr.istc.pst.oratio.gui;

import java.util.HashSet;
import java.util.Set;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.javalin.Javalin;
import io.javalin.websocket.WsContext;

public class App {

    static final Logger LOG = LoggerFactory.getLogger(App.class);
    private static Set<WsContext> contexts = new HashSet<>();

    public static void main(final String[] args) {
        final Javalin app = Javalin.create(config -> {
            config.addStaticFiles("/public");
        });
        app.ws("/solver", ws -> {
            ws.onConnect(ctx -> {
                LOG.info("New connection..");
                contexts.add(ctx);
            });
            ws.onClose(ctx -> {
                LOG.info("Lost connection..");
                contexts.remove(ctx);
            });
            ws.onMessage(ctx -> LOG.info("Received message {}..", ctx));
        });
        app.start();
    }
}
