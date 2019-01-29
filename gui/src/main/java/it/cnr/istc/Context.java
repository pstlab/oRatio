package it.cnr.istc;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Collection;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import it.cnr.istc.graph.CausalLinkAdded;
import it.cnr.istc.graph.CurrentFlaw;
import it.cnr.istc.graph.CurrentResolver;
import it.cnr.istc.graph.FlawCreated;
import it.cnr.istc.graph.FlawStateChanged;
import it.cnr.istc.graph.GraphListener;
import it.cnr.istc.graph.GraphScene;
import it.cnr.istc.graph.ResolverCostChanged;
import it.cnr.istc.graph.ResolverCreated;
import it.cnr.istc.graph.ResolverStateChanged;
import it.cnr.istc.riddle.Core;
import it.cnr.istc.riddle.CoreDeserializer;
import it.cnr.istc.state.StateListener;
import it.cnr.istc.state.StateScene;
import it.cnr.istc.timelines.TimelinesListener;
import it.cnr.istc.timelines.TimelinesScene;
import javafx.concurrent.Service;
import javafx.concurrent.Task;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;

/**
 * Context
 */
public class Context {

    private static Context context;

    private static final String FLAW_CREATED = "flaw_created ";
    private static final String FLAW_STATE_CHANGED = "flaw_state_changed ";
    private static final String CURRENT_FLAW = "current_flaw ";
    private static final String RESOLVER_CREATED = "resolver_created ";
    private static final String RESOLVER_STATE_CHANGED = "resolver_state_changed ";
    private static final String RESOLVER_COST_CHANGED = "resolver_cost_changed ";
    private static final String CURRENT_RESOLVER = "current_resolver ";
    private static final String CAUSAL_LINK_ADDED = "causal_link_added ";
    private static final String STATE_CHANGED = "state_changed ";

    private final ServerService service = new ServerService();

    /**
     * @return the context
     */
    public static Context getContext() {
        if (context == null)
            context = new Context();
        return context;
    }

    /**
     * @return the service
     */
    public ServerService getService() {
        return service;
    }

    public void showState() {
        Stage stage = new Stage();
        stage.setTitle("State");
        StateScene state_scene = new StateScene(service.getCore());
        service.addStateListener(state_scene);
        stage.setScene(state_scene);
        stage.setOnCloseRequest((WindowEvent event) -> service.removeStateListener(state_scene));
        stage.show();
    }

    public void showTimelines() {
        Stage stage = new Stage();
        stage.setTitle("Timelines");
        TimelinesScene timelines_scene = new TimelinesScene(service.getCore());
        service.addTimelinesListener(timelines_scene);
        stage.setScene(timelines_scene);
        stage.setOnCloseRequest((WindowEvent event) -> service.removeTimelinesListener(timelines_scene));
        stage.show();
    }

    public void showCausalGraph() {
        Stage stage = new Stage();
        stage.setTitle("Causal graph");
        GraphScene graph_scene = new GraphScene(service.getCore());
        service.addGraphListener(graph_scene);
        stage.setScene(graph_scene);
        stage.setOnCloseRequest((WindowEvent event) -> service.removeGraphListener(graph_scene));
        stage.show();
    }

    public static class ServerService extends Service<Boolean> {

        private final Core core = new Core();
        private final CoreDeserializer deserializer = new CoreDeserializer();
        private final Gson gson;
        private final Collection<GraphListener> graph_listeners = new ArrayList<>();
        private final Collection<StateListener> state_listeners = new ArrayList<>();
        private final Collection<TimelinesListener> timelines_listeners = new ArrayList<>();

        private ServerService() {
            final GsonBuilder builder = new GsonBuilder();
            builder.registerTypeAdapter(Core.class, deserializer);
            gson = builder.create();
        }

        /**
         * @return the core
         */
        public Core getCore() {
            return core;
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

        public void addTimelinesListener(TimelinesListener l) {
            timelines_listeners.add(l);
        }

        public void removeTimelinesListener(TimelinesListener l) {
            timelines_listeners.remove(l);
        }

        @Override
        protected Task<Boolean> createTask() {
            return new Task<Boolean>() {

                @Override
                protected Boolean call() throws Exception {
                    Logger.getLogger(Context.class.getName()).info("Starting server..");
                    try (ServerSocket ss = new ServerSocket(1100);
                            Socket client = ss.accept();
                            BufferedReader in = new BufferedReader(
                                    new InputStreamReader(client.getInputStream(), "UTF-8"))) {
                        String inputLine;
                        while ((inputLine = in.readLine()) != null) {
                            if (inputLine.startsWith(FLAW_CREATED)) {
                                FlawCreated fc = gson.fromJson(inputLine.substring(FLAW_CREATED.length()),
                                        FlawCreated.class);
                                for (GraphListener l : graph_listeners)
                                    l.flaw_created(fc);
                            } else if (inputLine.startsWith(FLAW_STATE_CHANGED)) {
                                FlawStateChanged fsc = gson.fromJson(inputLine.substring(FLAW_STATE_CHANGED.length()),
                                        FlawStateChanged.class);
                                for (GraphListener l : graph_listeners)
                                    l.flaw_state_changed(fsc);
                            } else if (inputLine.startsWith(CURRENT_FLAW)) {
                                CurrentFlaw cf = gson.fromJson(inputLine.substring(CURRENT_FLAW.length()),
                                        CurrentFlaw.class);
                                for (GraphListener l : graph_listeners)
                                    l.current_flaw(cf);
                            } else if (inputLine.startsWith(RESOLVER_CREATED)) {
                                ResolverCreated rc = gson.fromJson(inputLine.substring(RESOLVER_CREATED.length()),
                                        ResolverCreated.class);
                                for (GraphListener l : graph_listeners)
                                    l.resolver_created(rc);
                            } else if (inputLine.startsWith(RESOLVER_STATE_CHANGED)) {
                                ResolverStateChanged rsc = gson.fromJson(
                                        inputLine.substring(RESOLVER_STATE_CHANGED.length()),
                                        ResolverStateChanged.class);
                                for (GraphListener l : graph_listeners)
                                    l.resolver_state_changed(rsc);
                            } else if (inputLine.startsWith(RESOLVER_COST_CHANGED)) {
                                ResolverCostChanged rcc = gson.fromJson(
                                        inputLine.substring(RESOLVER_COST_CHANGED.length()), ResolverCostChanged.class);
                                for (GraphListener l : graph_listeners)
                                    l.resolver_cost_changed(rcc);
                            } else if (inputLine.startsWith(CURRENT_RESOLVER)) {
                                CurrentResolver cr = gson.fromJson(inputLine.substring(CURRENT_RESOLVER.length()),
                                        CurrentResolver.class);
                                for (GraphListener l : graph_listeners)
                                    l.current_resolver(cr);
                            } else if (inputLine.startsWith(CAUSAL_LINK_ADDED)) {
                                CausalLinkAdded cla = gson.fromJson(inputLine.substring(CAUSAL_LINK_ADDED.length()),
                                        CausalLinkAdded.class);
                                for (GraphListener l : graph_listeners)
                                    l.causal_link_added(cla);
                            } else if (inputLine.startsWith(STATE_CHANGED)) {
                                deserializer.setCore(core);
                                gson.fromJson(inputLine.substring(STATE_CHANGED.length()), Core.class);
                                for (StateListener l : state_listeners)
                                    l.stateChanged(core);
                                for (TimelinesListener l : timelines_listeners)
                                    l.timelinesChanged(core);
                            }
                        }
                    } catch (Exception ex) {
                        Logger.getLogger(Context.class.getName()).log(Level.SEVERE, null, ex);
                    }
                    return false;
                }
            };
        }
    }
}