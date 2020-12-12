package it.cnr.istc.pst.oratio;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonSubTypes.Type;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.oratio.Context.Message.CausalLinkAdded;
import it.cnr.istc.pst.oratio.Context.Message.CurrentFlaw;
import it.cnr.istc.pst.oratio.Context.Message.CurrentResolver;
import it.cnr.istc.pst.oratio.Context.Message.FlawCostChanged;
import it.cnr.istc.pst.oratio.Context.Message.FlawCreated;
import it.cnr.istc.pst.oratio.Context.Message.FlawPositionChanged;
import it.cnr.istc.pst.oratio.Context.Message.FlawStateChanged;
import it.cnr.istc.pst.oratio.Context.Message.Log;
import it.cnr.istc.pst.oratio.Context.Message.ReadFiles;
import it.cnr.istc.pst.oratio.Context.Message.ReadScript;
import it.cnr.istc.pst.oratio.Context.Message.ResolverCreated;
import it.cnr.istc.pst.oratio.Context.Message.ResolverStateChanged;
import it.cnr.istc.pst.oratio.Context.Message.StateChanged;
import it.cnr.istc.pst.oratio.riddle.Core;
import it.cnr.istc.pst.oratio.riddle.Rational;
import it.cnr.istc.pst.oratio.timelines.Timeline;
import it.cnr.istc.pst.oratio.timelines.TimelinesList;

public class Context {

    public static final int INF = Integer.MAX_VALUE / 2 - 1;
    private static final Logger LOG = LoggerFactory.getLogger(Context.class.getName());
    private static Context context;

    private final Core core = new Core();
    private final TimelinesList timelines = new TimelinesList(core);
    private final Collection<GraphListener> graph_listeners = new ArrayList<>();
    private final Collection<StateListener> state_listeners = new ArrayList<>();
    private final ObjectMapper mapper = new ObjectMapper();

    /**
     * @return the context
     */
    public static Context getContext() {
        if (context == null)
            context = new Context();
        return context;
    }

    private Context() {
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

    public void readState(File state) {
        try {
            mapper.readValue(state, Core.class);
            for (StateListener l : state_listeners)
                l.stateChanged(core);
        } catch (IOException e) {
            LOG.error("Failed at reading state", e);
        }
    }

    public void startServer(int port) {
        LOG.info("Starting the server..");
        while (true) {
            try (ServerSocket ss = new ServerSocket(port);
                    Socket client = ss.accept();
                    BufferedReader in = new BufferedReader(new InputStreamReader(client.getInputStream(), "UTF-8"))) {

                LOG.info("Initializing the core..");
                core.clear();
                for (StateListener l : state_listeners)
                    l.init();

                LOG.info("Waiting for input..");
                String inputLine;
                while ((inputLine = in.readLine()) != null) {
                    JsonNode tree_message = mapper.readTree(inputLine);
                    switch (tree_message.get("message_type").asText()) {
                        case "log": {
                            Log log = mapper.treeToValue(tree_message, Log.class);
                            for (StateListener l : state_listeners)
                                l.log(log.log);
                        }
                            break;
                        case "read_script": {
                            ReadScript read_script = mapper.treeToValue(tree_message, ReadScript.class);
                            core.read(read_script.script.replace("\\n", "\n"));
                        }
                            break;
                        case "read_files": {
                            ReadFiles read_files = mapper.treeToValue(tree_message, ReadFiles.class);
                            List<File> files = new ArrayList<>(read_files.files.length);
                            for (String file : read_files.files) {
                                File c_file = File.createTempFile("script_" + System.currentTimeMillis(), ".tmp");
                                try (BufferedWriter writer = new BufferedWriter(new FileWriter(c_file))) {
                                    writer.write(file.replace("\\n", "\n"));
                                }
                                files.add(c_file);
                            }
                            core.read(files.toArray(new File[files.size()]));
                        }
                            break;
                        case "state_changed": {
                            mapper.treeToValue(tree_message, StateChanged.class);
                            for (StateListener l : state_listeners)
                                l.stateChanged(core);
                        }
                            break;
                        case "flaw_created": {
                            FlawCreated flaw_created = mapper.treeToValue(tree_message, FlawCreated.class);
                            for (GraphListener l : graph_listeners)
                                l.flawCreated(flaw_created);
                        }
                            break;
                        case "flaw_state_changed": {
                            FlawStateChanged flaw_state_changed = mapper.treeToValue(tree_message,
                                    FlawStateChanged.class);
                            for (GraphListener l : graph_listeners)
                                l.flawStateChanged(flaw_state_changed);
                        }
                            break;
                        case "flaw_cost_changed": {
                            FlawCostChanged flaw_cost_changed = mapper.treeToValue(tree_message, FlawCostChanged.class);
                            for (GraphListener l : graph_listeners)
                                l.flawCostChanged(flaw_cost_changed);
                        }
                            break;
                        case "flaw_position_changed": {
                            FlawPositionChanged flaw_position_changed = mapper.treeToValue(tree_message,
                                    FlawPositionChanged.class);
                            for (GraphListener l : graph_listeners)
                                l.flawPositionChanged(flaw_position_changed);
                        }
                            break;
                        case "current_flaw": {
                            CurrentFlaw current_flaw = mapper.treeToValue(tree_message, CurrentFlaw.class);
                            for (GraphListener l : graph_listeners)
                                l.currentFlaw(current_flaw);
                        }
                            break;
                        case "resolver_created": {
                            ResolverCreated resolver_created = mapper.treeToValue(tree_message, ResolverCreated.class);
                            for (GraphListener l : graph_listeners)
                                l.resolverCreated(resolver_created);
                        }
                            break;
                        case "resolver_state_changed": {
                            ResolverStateChanged resolver_state_changed = mapper.treeToValue(tree_message,
                                    ResolverStateChanged.class);
                            for (GraphListener l : graph_listeners)
                                l.resolverStateChanged(resolver_state_changed);
                        }
                            break;
                        case "current_resolver": {
                            CurrentResolver current_resolver = mapper.treeToValue(tree_message, CurrentResolver.class);
                            for (GraphListener l : graph_listeners)
                                l.currentResolver(current_resolver);
                        }
                            break;
                        case "causal_link": {
                            CausalLinkAdded causal_link = mapper.treeToValue(tree_message, CausalLinkAdded.class);
                            for (GraphListener l : graph_listeners)
                                l.causalLinkAdded(causal_link);
                        }
                            break;
                        default:
                            LOG.info("Cannot handle message: {}", tree_message.get("message_type").asText());
                            break;
                    }
                }
                if (inputLine == null)
                    throw new RuntimeException("Connection lost..");
            } catch (Exception ex) {
                LOG.error("Cannot read socket", ex);
            }
        }
    }

    @SuppressWarnings({ "unused" })
    @JsonTypeInfo(use = JsonTypeInfo.Id.NAME, include = JsonTypeInfo.As.PROPERTY, property = "message_type")
    @JsonSubTypes({ @Type(value = Message.Log.class, name = "log"),
            @Type(value = Message.ReadScript.class, name = "read_script"),
            @Type(value = Message.ReadFiles.class, name = "read_files"),
            @Type(value = Message.StateChanged.class, name = "state_changed"),
            @Type(value = Message.FlawCreated.class, name = "flaw_created"),
            @Type(value = Message.FlawStateChanged.class, name = "flaw_state_changed"),
            @Type(value = Message.FlawCostChanged.class, name = "flaw_cost_changed"),
            @Type(value = Message.FlawPositionChanged.class, name = "flaw_position_changed"),
            @Type(value = Message.CurrentFlaw.class, name = "current_flaw"),
            @Type(value = Message.ResolverCreated.class, name = "resolver_created"),
            @Type(value = Message.ResolverStateChanged.class, name = "resolver_state_changed"),
            @Type(value = Message.CurrentResolver.class, name = "current_resolver"),
            @Type(value = Message.CausalLinkAdded.class, name = "causal_link") })
    public static abstract class Message {

        public static class Log extends Message {
            public String log;
        }

        public static class StateChanged extends Message {
            public Core state;
        }

        public static class ReadScript extends Message {
            public String script;
        }

        public static class ReadFiles extends Message {
            public String[] files;
        }

        public static class FlawCreated extends Message {
            public String id;
            public String[] causes;
            public String label;
            public int state;
            public Bound position;
        }

        public static class FlawStateChanged extends Message {
            public String id;
            public int state;
        }

        public static class FlawCostChanged extends Message {
            public String id;
            public Rational cost;
        }

        public static class FlawPositionChanged extends Message {
            public String id;
            public Bound position;
        }

        public static class CurrentFlaw extends Message {
            public String id;
        }

        public static class ResolverCreated extends Message {
            public String id;
            public String effect;
            public Rational cost;
            public String label;
            public int state;
        }

        public static class ResolverStateChanged extends Message {
            public String id;
            public int state;
        }

        public static class CurrentResolver extends Message {
            public String id;
        }

        public static class CausalLinkAdded extends Message {
            public String flaw;
            public String resolver;
        }
    }

    public static class Bound {

        public int min = -INF, max = INF;

        @Override
        public String toString() {
            if (min == max)
                return Integer.toString(min);

            String c_min = min == -INF ? "-inf" : Integer.toString(min);
            String c_max = max == INF ? "+inf" : Integer.toString(max);
            return "[" + c_min + ", " + c_max + "]";
        }
    }
}
