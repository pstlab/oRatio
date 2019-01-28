package it.cnr.istc.state;

import java.util.Arrays;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import it.cnr.istc.riddle.Atom;
import it.cnr.istc.riddle.Core;
import it.cnr.istc.riddle.Item;
import javafx.collections.ObservableList;
import javafx.scene.control.TreeItem;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

/**
 * StateNode
 */
public class StateNode extends TreeItem<String> {

    private static final ImageView CORE_ICON = new ImageView(
            new Image(StateNode.class.getResourceAsStream("/oRatio16.png")));
    private static final ImageView ACTIVE_FORMULA_ICON = new ImageView(
            new Image(StateNode.class.getResourceAsStream("/formula.png")));
    private static final ImageView INACTIVE_FORMULA_ICON = new ImageView(
            new Image(StateNode.class.getResourceAsStream("/inactive_formula.png")));
    private static final ImageView UNIFIED_FORMULA_ICON = new ImageView(
            new Image(StateNode.class.getResourceAsStream("/unified_formula.png")));
    private static final ImageView BOOL_ICON = new ImageView(
            new Image(StateNode.class.getResourceAsStream("/bool.png")));
    private static final ImageView ARITH_ICON = new ImageView(
            new Image(StateNode.class.getResourceAsStream("/number.png")));
    private static final ImageView STRING_ICON = new ImageView(
            new Image(StateNode.class.getResourceAsStream("/string.png")));
    private static final ImageView ENUM_ICON = new ImageView(
            new Image(StateNode.class.getResourceAsStream("/enum.png")));
    private static final ImageView ITEM_ICON = new ImageView(
            new Image(StateNode.class.getResourceAsStream("/object.png")));

    private final String name;
    private final Item item;
    private boolean hasLoadedChildren = false;

    StateNode(final String name, final Item item) {
        this.name = name;
        this.item = item;

        if (item == null) {
            setValue("Core");
            setGraphic(CORE_ICON);
        } else {
            if (item instanceof Atom) {
                setValue(name);
                switch (((Atom) item).getState()) {
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
                    throw new AssertionError(((Atom) item).getState().name());
                }
            } else {
                switch (item.getType().getName()) {
                case Core.BOOL:
                    setGraphic(BOOL_ICON);
                    setValue(name + " = " + ((Item.BoolItem) item).getValue());
                    setToolTipText(((Item.BoolItem) item).getLit());
                    break;
                case Core.INT:
                case Core.REAL:
                    setGraphic(ARITH_ICON);
                    setValue(name + " = " + ((Item.ArithItem) item).getValue());
                    setToolTipText(((Item.ArithItem) item).getLin() + " = [" + ((Item.ArithItem) item).getLb() + ", "
                            + ((Item.ArithItem) item).getUb() + "]");
                    break;
                case Core.STRING:
                    setGraphic(STRING_ICON);
                    setValue(name + " = " + ((Item.StringItem) item).getValue());
                    break;
                default:
                    setValue(name);
                    if (item instanceof Item.EnumItem) {
                        setGraphic(ENUM_ICON);
                    } else {
                        setGraphic(ITEM_ICON);
                    }
                }
            }
        }
    }

    @Override
    public boolean isLeaf() {
        if (item == null) {
            return false;
        } else {
            switch (item.getType().getName()) {
            case Core.BOOL:
            case Core.INT:
            case Core.REAL:
            case Core.STRING:
                return true;
            default:
                return false;
            }
        }
    }

    @Override
    public ObservableList<TreeItem<String>> getChildren() {
        if (!hasLoadedChildren) {
            super.getChildren().setAll(Stream.concat(
                    item.getExprs().entrySet().stream().map(xpr -> new StateNode(xpr.getKey(), xpr.getValue())),
                    item.getType().getPredicates().values().stream().flatMap(p -> p.getInstances().stream()
                            .filter(i -> (((Atom) i).getTau() == item || (((Atom) i).getTau() instanceof Item.EnumItem
                                    && Arrays.asList(((Item.EnumItem) ((Atom) i).getTau()).getVals()).contains(item))))
                            .map(i -> new StateNode(((Atom) i).getType().getName(), i))))
                    .collect(Collectors.toList()));
            hasLoadedChildren = true;
        }
        return super.getChildren();
    }
}