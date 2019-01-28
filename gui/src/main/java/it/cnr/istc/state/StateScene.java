package it.cnr.istc.state;

import java.util.stream.Collectors;

import it.cnr.istc.riddle.Core;
import javafx.scene.Scene;
import javafx.scene.control.TreeView;

/**
 * StateScene
 */
public class StateScene extends Scene implements StateListener {

    private final TreeView<String> tree;
    private final StateNode rootItem = new StateNode("Core", null);

    @SuppressWarnings("unchecked")
    public StateScene() {
        super(new TreeView<String>());
        tree = (TreeView<String>) getRoot();
        tree.setRoot(rootItem);
        rootItem.setExpanded(true);
    }

    @Override
    public void stateChanged(Core core) {
        rootItem.getChildren().setAll(core.getExprs().entrySet().stream()
                .map(xpr -> new StateNode(xpr.getKey(), xpr.getValue())).collect(Collectors.toList()));
    }
}