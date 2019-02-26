package it.cnr.istc;

import it.cnr.istc.oratio.StateListener;
import it.cnr.istc.oratio.riddle.Core;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.scene.Scene;
import javafx.scene.control.ListView;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;

/**
 * MainScene
 */
public class MainScene extends Scene implements StateListener {

    private final ObservableList<String> logs = FXCollections.observableArrayList();
    private final ListView<String> list = new ListView<>(logs);

    public MainScene() {
        super(new VBox(), 600, 400);
        ((VBox) getRoot()).getChildren().add(list);
        VBox.setVgrow(list, Priority.ALWAYS);
    }

    @Override
    public void log(String log) {
        logs.add(log);
    }

    @Override
    public void stateChanged(Core core) {
    }
}