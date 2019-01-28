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
        stage.setTitle("oRatio");
        stage.setScene(new MainScene());
        stage.setOnCloseRequest((WindowEvent event) -> Platform.exit());
        stage.show();

        Stage state_stage = new Stage();
        state_stage.setTitle("State");
        state_stage.setScene(new StateScene());
        state_stage.show();

        Stage timelines_stage = new Stage();
        timelines_stage.setTitle("Timelines");
        timelines_stage.setScene(new TimelinesScene());
        timelines_stage.show();

        Stage graph_stage = new Stage();
        graph_stage.setTitle("Causal graph");
        GraphScene graph_scene = new GraphScene();
        graph_stage.setScene(graph_scene);
        graph_stage.show();

        Context.getContext().getService().getCore()
                .read(getParameters().getUnnamed().stream().map(arg -> new File(arg)).toArray(File[]::new));
        Context.getContext().getService().addGraphListener(graph_scene);
        Context.getContext().getService().start();
    }
}