package it.cnr.istc;

import java.io.File;

import it.cnr.istc.graph.GraphScene;
import it.cnr.istc.oratio.Context;
import it.cnr.istc.state.StateScene;
import it.cnr.istc.timelines.TimelinesScene;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.concurrent.Service;
import javafx.concurrent.Task;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;

/**
 * App
 */
public class App extends Application {

    private final ServerService service = new ServerService();

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage stage) throws Exception {
        Context.getContext().getCore()
                .read(getParameters().getUnnamed().stream().map(arg -> new File(arg)).toArray(File[]::new));

        stage.setTitle("oRatio");
        MainScene main_scene = new MainScene();
        Context.getContext().addStateListener(main_scene);
        stage.setScene(main_scene);
        stage.setOnCloseRequest((WindowEvent event) -> Platform.exit());
        stage.show();

        showState();
        showTimelines();
        showCausalGraph();

        service.start();
    }

    private void showState() {
        Stage stage = new Stage();
        stage.setTitle("State");
        StateScene state_scene = new StateScene(Context.getContext().getCore());
        Context.getContext().addStateListener(state_scene);
        stage.setScene(state_scene);
        stage.setOnCloseRequest((WindowEvent event) -> Context.getContext().removeStateListener(state_scene));
        stage.show();
    }

    private void showTimelines() {
        Stage stage = new Stage();
        stage.setTitle("Timelines");
        TimelinesScene timelines_scene = new TimelinesScene(Context.getContext().getCore());
        Context.getContext().addStateListener(timelines_scene);
        stage.setScene(timelines_scene);
        stage.setOnCloseRequest((WindowEvent event) -> Context.getContext().removeStateListener(timelines_scene));
        stage.show();
    }

    private void showCausalGraph() {
        Stage stage = new Stage();
        stage.setTitle("Causal graph");
        GraphScene graph_scene = new GraphScene(Context.getContext().getCore());
        Context.getContext().addGraphListener(graph_scene);
        stage.setScene(graph_scene);
        stage.setOnCloseRequest((WindowEvent event) -> Context.getContext().removeGraphListener(graph_scene));
        stage.show();
    }

    private static class ServerService extends Service<Boolean> {

        @Override
        protected Task<Boolean> createTask() {
            return new Task<Boolean>() {

                @Override
                protected Boolean call() throws Exception {
                    Context.getContext().startServer();
                    return false;
                }
            };
        }
    }
}