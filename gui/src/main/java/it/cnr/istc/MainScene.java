package it.cnr.istc;

import it.cnr.istc.oratio.StateListener;
import it.cnr.istc.oratio.riddle.Core;
import javafx.scene.Scene;
import javafx.scene.layout.VBox;

/**
 * MainScene
 */
public class MainScene extends Scene implements StateListener {

    public MainScene() {
        super(new VBox(), 400, 300);
    }

    @Override
    public void log(String log) {
    }

    @Override
    public void stateChanged(Core core) {
    }
}