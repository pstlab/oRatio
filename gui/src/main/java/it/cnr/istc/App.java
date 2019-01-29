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

        Context.getContext().showState();
        Context.getContext().showTimelines();
        Context.getContext().showCausalGraph();

        Context.getContext().getService().start();
    }
}