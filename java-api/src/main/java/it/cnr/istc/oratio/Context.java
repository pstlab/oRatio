package it.cnr.istc.oratio;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;

import it.cnr.istc.oratio.GraphListener.*;
import it.cnr.istc.oratio.riddle.Core;
import it.cnr.istc.oratio.riddle.CoreDeserializer;
import it.cnr.istc.oratio.timelines.Timeline;
import it.cnr.istc.oratio.timelines.TimelinesList;

/**
 * Context
 */
public class Context {

    private static final Logger LOG = Logger.getLogger(Context.class.getName());
    private static Context context;

    private static final String LOG_MESSAGE = "log ";
    private static final String STATE_CHANGED = "state_changed ";
    private static final String READ_0 = "read0 ";
    private static final String READ_1 = "read1 ";

    private static final String FLAW_CREATED = "flaw_created ";
    private static final String FLAW_STATE_CHANGED = "flaw_state_changed ";
    private static final String FLAW_COST_CHANGED = "flaw_cost_changed ";
    private static final String CURRENT_FLAW = "current_flaw ";
    private static final String RESOLVER_CREATED = "resolver_created ";
    private static final String RESOLVER_STATE_CHANGED = "resolver_state_changed ";
    private static final String RESOLVER_COST_CHANGED = "resolver_cost_changed ";
    private static final String CURRENT_RESOLVER = "current_resolver ";
    private static final String CAUSAL_LINK_ADDED = "causal_link_added ";
    private static final String POKE = "poke";

    private final Core core = new Core();
    private final TimelinesList timelines = new TimelinesList(core);
    private final Collection<GraphListener> graph_listeners = new ArrayList<>();
    private final Collection<StateListener> state_listeners = new ArrayList<>();
    private final Gson gson;
    private final CoreDeserializer deserializer = new CoreDeserializer();

    /**
     * @return the context
     */
    public static Context getContext() {
        if (context == null)
            context = new Context();
        return context;
    }

    private Context() {
        final GsonBuilder builder = new GsonBuilder();
        builder.registerTypeAdapter(Core.class, deserializer);
        gson = builder.create();

        state_listeners.add(timelines);
    }

    /**
     * @return the core
     */
    public Core getCore() {
        return core;
    }

    /**
     * @return the timelines
     */
    public List<Timeline<?>> getTimelines() {
        return Collections.unmodifiableList(timelines);
    }

    public void addGraphListener(GraphListener l) {
        graph_listeners.add(l);
    }

    public void removeGraphListener(GraphListener l) {
        graph_listeners.remove(l);
    }

    public void addStateListener(StateListener l) {
        state_listeners.add(l);
    }

    public void removeStateListener(StateListener l) {
        state_listeners.remove(l);
    }

    public void startServer() {
        LOG.info("Starting server..");
        try (ServerSocket ss = new ServerSocket(1100);
                Socket client = ss.accept();
                BufferedReader in = new BufferedReader(new InputStreamReader(client.getInputStream(), "UTF-8"))) {
            String inputLine;
            while ((inputLine = in.readLine()) != null) {
                if (inputLine.startsWith(LOG_MESSAGE)) {
                    for (StateListener l : state_listeners)
                        l.log(inputLine.substring(LOG_MESSAGE.length()));
                } else if (inputLine.startsWith(STATE_CHANGED)) {
                    deserializer.setCore(core);
                    gson.fromJson(inputLine.substring(STATE_CHANGED.length()), Core.class);
                    for (StateListener l : state_listeners)
                        l.stateChanged(core);
                } else if (inputLine.startsWith(READ_0)) {
                    StringBuilder script = new StringBuilder();
                    script.append(inputLine.substring(READ_0.length()));
                    while ((inputLine = in.readLine()) != null && inputLine.equals("EOS")) {
                        script.append('\n').append(inputLine);
                    }
                    core.read(script.toString());
                    for (StateListener l : state_listeners)
                        l.stateChanged(core);
                } else if (inputLine.startsWith(READ_1)) {
                    deserializer.setCore(core);
                    List<String> files = gson.fromJson(inputLine.substring(STATE_CHANGED.length()),
                            new TypeToken<List<String>>() {
                            }.getType());
                    core.read(files.toArray(String[]::new));
                    for (StateListener l : state_listeners)
                        l.stateChanged(core);
                } else if (inputLine.startsWith(FLAW_CREATED)) {
                    FlawCreated fc = gson.fromJson(inputLine.substring(FLAW_CREATED.length()), FlawCreated.class);
                    for (GraphListener l : graph_listeners)
                        l.flawCreated(fc);
                } else if (inputLine.startsWith(FLAW_STATE_CHANGED)) {
                    FlawStateChanged fsc = gson.fromJson(inputLine.substring(FLAW_STATE_CHANGED.length()),
                            FlawStateChanged.class);
                    for (GraphListener l : graph_listeners)
                        l.flawStateChanged(fsc);
                } else if (inputLine.startsWith(FLAW_COST_CHANGED)) {
                    FlawCostChanged fsc = gson.fromJson(inputLine.substring(FLAW_COST_CHANGED.length()),
                            FlawCostChanged.class);
                    for (GraphListener l : graph_listeners)
                        l.flawCostChanged(fsc);
                } else if (inputLine.startsWith(CURRENT_FLAW)) {
                    CurrentFlaw cf = gson.fromJson(inputLine.substring(CURRENT_FLAW.length()), CurrentFlaw.class);
                    for (GraphListener l : graph_listeners)
                        l.currentFlaw(cf);
                } else if (inputLine.startsWith(RESOLVER_CREATED)) {
                    ResolverCreated rc = gson.fromJson(inputLine.substring(RESOLVER_CREATED.length()),
                            ResolverCreated.class);
                    for (GraphListener l : graph_listeners)
                        l.resolverCreated(rc);
                } else if (inputLine.startsWith(RESOLVER_STATE_CHANGED)) {
                    ResolverStateChanged rsc = gson.fromJson(inputLine.substring(RESOLVER_STATE_CHANGED.length()),
                            ResolverStateChanged.class);
                    for (GraphListener l : graph_listeners)
                        l.resolverStateChanged(rsc);
                } else if (inputLine.startsWith(RESOLVER_COST_CHANGED)) {
                    ResolverCostChanged rcc = gson.fromJson(inputLine.substring(RESOLVER_COST_CHANGED.length()),
                            ResolverCostChanged.class);
                    for (GraphListener l : graph_listeners)
                        l.resolverCostChanged(rcc);
                } else if (inputLine.startsWith(CURRENT_RESOLVER)) {
                    CurrentResolver cr = gson.fromJson(inputLine.substring(CURRENT_RESOLVER.length()),
                            CurrentResolver.class);
                    for (GraphListener l : graph_listeners)
                        l.currentResolver(cr);
                } else if (inputLine.startsWith(CAUSAL_LINK_ADDED)) {
                    CausalLinkAdded cla = gson.fromJson(inputLine.substring(CAUSAL_LINK_ADDED.length()),
                            CausalLinkAdded.class);
                    for (GraphListener l : graph_listeners)
                        l.causalLinkAdded(cla);
                } else if (inputLine.startsWith(POKE)) {
                    for (GraphListener l : graph_listeners)
                        l.poke();
                } else
                    LOG.warning("Cannot handle message: " + inputLine);
            }
        } catch (Exception ex) {
            LOG.log(Level.SEVERE, null, ex);
        }
    }
}