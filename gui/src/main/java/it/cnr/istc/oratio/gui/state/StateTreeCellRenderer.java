/*
 * Copyright (C) 2018 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package it.cnr.istc.oratio.gui.state;

import it.cnr.istc.oratio.gui.riddle.Atom;
import it.cnr.istc.oratio.gui.riddle.Core;
import it.cnr.istc.oratio.gui.riddle.Item;
import java.awt.Component;
import javax.swing.ImageIcon;
import javax.swing.JTree;
import javax.swing.tree.DefaultTreeCellRenderer;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class StateTreeCellRenderer extends DefaultTreeCellRenderer {

    private static final ImageIcon CORE_ICON = new ImageIcon(StateTreeCellRenderer.class.getResource("/oRatio16.png"));
    private static final ImageIcon ACTIVE_FORMULA_ICON = new ImageIcon(StateTreeCellRenderer.class.getResource("/formula.png"));
    private static final ImageIcon INACTIVE_FORMULA_ICON = new ImageIcon(StateTreeCellRenderer.class.getResource("/inactive_formula.png"));
    private static final ImageIcon UNIFIED_FORMULA_ICON = new ImageIcon(StateTreeCellRenderer.class.getResource("/unified_formula.png"));
    private static final ImageIcon BOOL_ICON = new ImageIcon(StateTreeCellRenderer.class.getResource("/bool.png"));
    private static final ImageIcon ARITH_ICON = new ImageIcon(StateTreeCellRenderer.class.getResource("/number.png"));
    private static final ImageIcon STRING_ICON = new ImageIcon(StateTreeCellRenderer.class.getResource("/string.png"));
    private static final ImageIcon ENUM_ICON = new ImageIcon(StateTreeCellRenderer.class.getResource("/enum.png"));
    private static final ImageIcon ITEM_ICON = new ImageIcon(StateTreeCellRenderer.class.getResource("/object.png"));

    @Override
    public Component getTreeCellRendererComponent(JTree tree, Object value, boolean sel, boolean expanded, boolean leaf, int row, boolean hasFocus) {
        super.getTreeCellRendererComponent(tree, value, sel, expanded, leaf, row, hasFocus);
        StateTreeNode node = (StateTreeNode) value;
        if (node.getUserObject() == null) {
            setText("Core");
            setIcon(CORE_ICON);
        } else {
            Item item = (Item) node.getUserObject();
            if (item instanceof Atom) {
                setText(node.name);
                switch (((Atom) item).getState()) {
                    case Active:
                        setIcon(ACTIVE_FORMULA_ICON);
                        break;
                    case Inactive:
                        setIcon(INACTIVE_FORMULA_ICON);
                        break;
                    case Unified:
                        setIcon(UNIFIED_FORMULA_ICON);
                        break;
                    default:
                        throw new AssertionError(((Atom) item).getState().name());
                }
            } else {
                switch (item.getType().getName()) {
                    case Core.BOOL:
                        setIcon(BOOL_ICON);
                        setText(node.name + " = " + ((Item.BoolItem) item).getValue());
                        setToolTipText(((Item.BoolItem) item).getLit());
                        break;
                    case Core.INT:
                    case Core.REAL:
                        setIcon(ARITH_ICON);
                        setText(node.name + " = " + ((Item.ArithItem) item).getValue());
                        setToolTipText(((Item.ArithItem) item).getLin() + " = [" + ((Item.ArithItem) item).getLb() + ", " + ((Item.ArithItem) item).getUb() + "]");
                        break;
                    case Core.STRING:
                        setIcon(STRING_ICON);
                        setText(node.name + " = " + ((Item.StringItem) item).getValue());
                        break;
                    default:
                        setText(node.name);
                        if (item instanceof Item.EnumItem) {
                            setIcon(ENUM_ICON);
                        } else {
                            setIcon(ITEM_ICON);
                        }
                }
            }
        }
        return this;
    }
}
