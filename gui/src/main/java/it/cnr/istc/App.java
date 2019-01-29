package it.cnr.istc;

import java.io.File;

import it.cnr.istc.graph.GraphScene;
import it.cnr.istc.state.StateScene;
import it.cnr.istc.timelines.TimelinesScene;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;

/**
 * App
 */
public class App extends Application {

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage stage) throws Exception {
        Context.getContext().getService().getCore()
                .read(getParameters().getUnnamed().stream().map(arg -> new File(arg)).toArray(File[]::new));

        stage.setTitle("oRatio");
        stage.setScene(new MainScene());
        stage.setOnCloseRequest((WindowEvent event) -> Platform.exit());
        stage.show();

        Stage state_stage = new Stage();
        state_stage.setTitle("State");
        StateScene state_scene = new StateScene(Context.getContext().getService().getCore());
        Context.getContext().getService().addStateListener(state_scene);
        state_stage.setScene(state_scene);
        state_stage.setOnCloseRequest(
                (WindowEvent event) -> Context.getContext().getService().removeStateListener(state_scene));
        state_stage.show();

        Stage timelines_stage = new Stage();
        timelines_stage.setTitle("Timelines");
        timelines_stage.setScene(new TimelinesScene(Context.getContext().getService().getCore()));
        timelines_stage.show();

        Stage graph_stage = new Stage();
        graph_stage.setTitle("Causal graph");
        GraphScene graph_scene = new GraphScene(Context.getContext().getService().getCore());
        Context.getContext().getService().addGraphListener(graph_scene);
        graph_stage.setScene(graph_scene);
        graph_stage.show();

        Context.getContext().getService().start();
    }
}