package it.cnr.istc.state;

import java.util.Arrays;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import it.cnr.istc.riddle.Atom;
import it.cnr.istc.riddle.Core;
import it.cnr.istc.riddle.Item;
import javafx.collections.ObservableList;
import javafx.scene.Scene;
import javafx.scene.control.Tooltip;
import javafx.scene.control.TreeCell;
import javafx.scene.control.TreeItem;
import javafx.scene.control.TreeView;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

/**
 * StateScene
 */
public class StateScene extends Scene implements StateListener {

    private static final ImageView CORE_ICON = new ImageView(
            new Image(StateScene.class.getResourceAsStream("/oRatio16.png")));
    private static final ImageView ACTIVE_FORMULA_ICON = new ImageView(
            new Image(StateScene.class.getResourceAsStream("/formula.png")));
    private static final ImageView INACTIVE_FORMULA_ICON = new ImageView(
            new Image(StateScene.class.getResourceAsStream("/inactive_formula.png")));
    private static final ImageView UNIFIED_FORMULA_ICON = new ImageView(
            new Image(StateScene.class.getResourceAsStream("/unified_formula.png")));
    private static final ImageView BOOL_ICON = new ImageView(
            new Image(StateScene.class.getResourceAsStream("/bool.png")));
    private static final ImageView ARITH_ICON = new ImageView(
            new Image(StateScene.class.getResourceAsStream("/number.png")));
    private static final ImageView STRING_ICON = new ImageView(
            new Image(StateScene.class.getResourceAsStream("/string.png")));
    private static final ImageView ENUM_ICON = new ImageView(
            new Image(StateScene.class.getResourceAsStream("/enum.png")));
    private static final ImageView ITEM_ICON = new ImageView(
            new Image(StateScene.class.getResourceAsStream("/object.png")));
    private final Core core;
    private final TreeItem<StateNode> root = new TreeItem<StateNode>(null, CORE_ICON);

    @SuppressWarnings("unchecked")
    public StateScene(final Core core) {
        super(new TreeView<StateNode>());
        this.core = core;
        ((TreeView<StateNode>) getRoot()).setRoot(root);
        ((TreeView<StateNode>) getRoot()).setCellFactory(tv -> {
            final Tooltip tooltip = new Tooltip();
            TreeCell<StateNode> cell = new TreeCell<>() {

                @Override
                public void updateItem(StateNode item, boolean empty) {
                    super.updateItem(item, empty);
                    if (empty) {
                        setText(null);
                        setTooltip(null);
                    } else if (getTreeItem() == root) {
                        setText("Core");
                        setGraphic(CORE_ICON);
                        setTooltip(null);
                    } else if (item.item instanceof Atom) {
                        setText(item.name);
                        switch (((Atom) item.item).getState()) {
                        case Active:
                            setGraphic(ACTIVE_FORMULA_ICON);
                            break;
                        case Inactive:
                            setGraphic(INACTIVE_FORMULA_ICON);
                            break;
                        case Unified:
                            setGraphic(UNIFIED_FORMULA_ICON);
                            break;
                        default:
                            throw new AssertionError(((Atom) item.item).getState().name());
                        }
                        StringBuilder sb = new StringBuilder();
                        sb.append(item.item.getType().getName()).append("(");
                        for (Map.Entry<String, Item> expr : item.item.getExprs().entrySet()) {
                            sb.append(", ");
                            switch (expr.getValue().getType().getName()) {
                            case Core.BOOL:
                                sb.append(expr.getKey()).append(" = ")
                                        .append(((Item.BoolItem) expr.getValue()).getValue());
                                break;
                            case Core.INT:
                            case Core.REAL:
                                sb.append(expr.getKey()).append(" = ")
                                        .append(((Item.ArithItem) expr.getValue()).getValue());
                                break;
                            case Core.STRING:
                                sb.append(expr.getKey()).append(" = ")
                                        .append(((Item.StringItem) expr.getValue()).getValue());
                                break;
                            default:
                                sb.append(expr.getKey());
                            }
                        }
                        sb.append(")");
                        tooltip.setText(sb.toString().replace("(, ", "("));
                        setTooltip(tooltip);
                    } else {
                        switch (item.item.getType().getName()) {
                        case Core.BOOL:
                            setGraphic(BOOL_ICON);
                            setText(item.name + " = " + ((Item.BoolItem) item.item).getValue());
                            tooltip.setText(((Item.BoolItem) item.item).getLit());
                            setTooltip(tooltip);
                            break;
                        case Core.INT:
                        case Core.REAL:
                            setGraphic(ARITH_ICON);
                            setText(item.name + " = " + ((Item.ArithItem) item.item).getValue());
                            tooltip.setText(((Item.ArithItem) item.item).getLin() + " = ["
                                    + ((Item.ArithItem) item.item).getLb() + ", " + ((Item.ArithItem) item.item).getUb()
                                    + "]");
                            setTooltip(tooltip);
                            break;
                        case Core.STRING:
                            setGraphic(STRING_ICON);
                            setText(item.name + " = " + ((Item.StringItem) item.item).getValue());
                            break;
                        default:
                            setText(item.name);
                            if (item.item instanceof Item.EnumItem) {
                                setGraphic(ENUM_ICON);
                            } else {
                                setGraphic(ITEM_ICON);
                            }
                        }
                    }
                }
            };
            return cell;
        });
        root.setExpanded(true);
    }

    @Override
    public void stateChanged(Core core) {
        // TODO: update nodes instead of replacing..
        root.getChildren()
                .setAll(core.getExprs().entrySet().stream()
                        .map(xpr -> new StateTreeItem(new StateNode(xpr.getKey(), xpr.getValue())))
                        .collect(Collectors.toList()));
    }

    private static class StateNode {

        private final String name;
        private final Item item;

        private StateNode(final String name, final Item item) {
            this.name = name;
            this.item = item;
        }
    }

    private static class StateTreeItem extends TreeItem<StateNode> {

        private boolean hasLoadedChildren = false;

        private StateTreeItem(StateNode node) {
            super(node);
        }

        @Override
        public boolean isLeaf() {
            switch (getValue().item.getType().getName()) {
            case Core.BOOL:
            case Core.INT:
            case Core.REAL:
            case Core.STRING:
                return true;
            default:
                return false;
            }
        }

        @Override
        public ObservableList<TreeItem<StateNode>> getChildren() {
            if (!hasLoadedChildren) {
                // TODO: update nodes instead of replacing..
                super.getChildren().setAll(Stream
                        .concat(getValue().item
                                .getExprs().entrySet().stream()
                                .map(xpr -> new StateTreeItem(new StateNode(xpr.getKey(), xpr.getValue()))),
                                getValue().item.getType().getPredicates().values().stream()
                                        .flatMap(p -> p.getInstances().stream()
                                                .filter(i -> (((Atom) i).getTau() == getValue().item
                                                        || (((Atom) i).getTau() instanceof Item.EnumItem && Arrays
                                                                .asList(((Item.EnumItem) ((Atom) i).getTau()).getVals())
                                                                .contains(getValue().item))))
                                                .map(i -> new StateTreeItem(
                                                        new StateNode(((Atom) i).getType().getName(), i)))))
                        .collect(Collectors.toList()));
                hasLoadedChildren = true;
            }
            return super.getChildren();
        }
    }
}