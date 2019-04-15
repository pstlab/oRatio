package it.cnr.istc.oratio.gui;

import java.awt.Component;
import java.util.Arrays;

import javax.swing.ImageIcon;
import javax.swing.JInternalFrame;
import javax.swing.JScrollPane;
import javax.swing.JTree;
import javax.swing.event.TreeExpansionEvent;
import javax.swing.event.TreeModelEvent;
import javax.swing.event.TreeModelListener;
import javax.swing.event.TreeWillExpandListener;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeCellRenderer;
import javax.swing.tree.DefaultTreeModel;
import javax.swing.tree.ExpandVetoException;
import javax.swing.tree.TreeModel;

import it.cnr.istc.oratio.StateListener;
import it.cnr.istc.oratio.riddle.Atom;
import it.cnr.istc.oratio.riddle.Core;
import it.cnr.istc.oratio.riddle.Item;

/**
 * StateJInternalFrame
 */
public class StateJInternalFrame extends JInternalFrame implements StateListener {

    private static final ImageIcon CORE_ICON = new ImageIcon(StateJInternalFrame.class.getResource("/oRatio16.png"));
    private static final ImageIcon ACTIVE_ATOM_ICON = new ImageIcon(StateJInternalFrame.class.getResource("/atom.png"));
    private static final ImageIcon INACTIVE_ATOM_ICON = new ImageIcon(
            StateJInternalFrame.class.getResource("/inactive_atom.png"));
    private static final ImageIcon UNIFIED_ATOM_ICON = new ImageIcon(
            StateJInternalFrame.class.getResource("/unified_atom.png"));
    private static final ImageIcon BOOL_ICON = new ImageIcon(StateJInternalFrame.class.getResource("/bool.png"));
    private static final ImageIcon ARITH_ICON = new ImageIcon(StateJInternalFrame.class.getResource("/number.png"));
    private static final ImageIcon STRING_ICON = new ImageIcon(StateJInternalFrame.class.getResource("/string.png"));
    private static final ImageIcon ENUM_ICON = new ImageIcon(StateJInternalFrame.class.getResource("/enum.png"));
    private static final ImageIcon ITEM_ICON = new ImageIcon(StateJInternalFrame.class.getResource("/object.png"));

    private final Core core;
    private final StateNode root;
    private final TreeModel tree_model;
    private final JTree state_tree;

    public StateJInternalFrame(final Core core) {
        super("State");
        this.core = core;

        root = new StateNode("oRatio", core);
        tree_model = new DefaultTreeModel(root);
        tree_model.addTreeModelListener(new TreeModelListener() {

            @Override
            public void treeStructureChanged(TreeModelEvent e) {
            }

            @Override
            public void treeNodesRemoved(TreeModelEvent e) {
            }

            @Override
            public void treeNodesInserted(TreeModelEvent e) {
            }

            @Override
            public void treeNodesChanged(TreeModelEvent e) {
                state_tree.expandRow(0);
            }
        });
        state_tree = new JTree(tree_model);
        state_tree.setCellRenderer(new DefaultTreeCellRenderer() {
            @Override
            public Component getTreeCellRendererComponent(JTree tree, Object value, boolean sel, boolean expanded,
                    boolean leaf, int row, boolean hasFocus) {
                super.getTreeCellRendererComponent(tree, value, sel, leaf, leaf, row, leaf);
                StateNode node = (StateNode) value;
                if (node.getUserObject() instanceof Core) {
                    setText(node.name);
                } else if (node.getUserObject() instanceof Atom) {
                    setText(node.name);
                    switch (((Atom) node.getUserObject()).getState()) {
                    case Active:
                        setIcon(ACTIVE_ATOM_ICON);
                        break;
                    case Inactive:
                        setIcon(INACTIVE_ATOM_ICON);
                        break;
                    case Unified:
                        setIcon(UNIFIED_ATOM_ICON);
                        break;
                    default:
                        throw new AssertionError(((Atom) node.getUserObject()).getState().name());
                    }
                    setToolTipText(((Atom) node.getUserObject()).toString());
                } else {
                    switch (((Item) node.getUserObject()).getType().getName()) {
                    case "bool":
                        setText(node.name + " = " + ((Item.BoolItem) node.getUserObject()).getValue());
                        setIcon(BOOL_ICON);
                        setToolTipText(((Item.BoolItem) node.getUserObject()).getLit());
                        break;
                    case "int":
                    case "real":
                        setText(node.name + " = " + ((Item.ArithItem) node.getUserObject()).getValue());
                        setIcon(ARITH_ICON);
                        setToolTipText(((Item.ArithItem) node.getUserObject()).getLin() + " = ["
                                + ((Item.ArithItem) node.getUserObject()).getLb() + ", "
                                + ((Item.ArithItem) node.getUserObject()).getUb() + "]");
                        break;
                    case "string":
                        setText(node.name + " = " + ((Item.StringItem) node.getUserObject()).getValue());
                        setIcon(STRING_ICON);
                        setToolTipText(null);
                        break;
                    default:
                        setText(node.name);
                        if (node.getUserObject() instanceof Item.EnumItem)
                            setIcon(ENUM_ICON);
                        else
                            setIcon(ITEM_ICON);
                        setToolTipText(null);
                    }
                }
                return this;
            }
        });
        state_tree.addTreeWillExpandListener(new TreeWillExpandListener() {

            @Override
            public void treeWillExpand(TreeExpansionEvent event) throws ExpandVetoException {
                StateNode node = (StateNode) event.getPath().getLastPathComponent();
                if (!node.hasLoadedChildren) {
                    node.loadChildren();
                }
            }

            @Override
            public void treeWillCollapse(TreeExpansionEvent event) throws ExpandVetoException {
            }
        });
        state_tree.setRootVisible(false);
        add(new JScrollPane(state_tree));

        setIconifiable(true);
        setMaximizable(true);
        setResizable(true);
        pack();
    }

    @Override
    public void log(String log) {

    }

    @Override
    public void stateChanged(Core core) {
        root.removeAllChildren();
        root.hasLoadedChildren = false;
        root.loadChildren();
    }

    private static class StateNode extends DefaultMutableTreeNode {

        private final String name;
        private boolean hasLoadedChildren = false;

        private StateNode(final String name, final Object item) {
            super(item);
            this.name = name;
        }

        @Override
        public boolean isLeaf() {
            if (userObject instanceof Core)
                return false;
            else
                switch (((Item) userObject).getType().getName()) {
                case "bool":
                case "int":
                case "real":
                case "string":
                    return true;
                default:
                    return false;
                }
        }

        @Override
        public boolean getAllowsChildren() {
            if (userObject instanceof Core)
                return !((Core) userObject).getExprs().isEmpty() || (((Core) userObject).getPredicates().values()
                        .stream().flatMap(pred -> pred.getInstances().stream()).count() > 0);
            else
                return !((Item) userObject).getExprs().isEmpty() || (((Item) userObject).getType().getPredicates()
                        .values().stream().flatMap(pred -> pred.getInstances().stream()).count() > 0);
        }

        void loadChildren() {
            if (!hasLoadedChildren) {
                if (userObject instanceof Core) {
                    ((Core) userObject).getExprs().entrySet()
                            .forEach(expr -> add(new StateNode(expr.getKey(), expr.getValue())));
                    ((Core) userObject).getPredicates().values().stream().flatMap(pred -> pred.getInstances().stream())
                            .filter(atm -> (((Atom) atm).getTau() == userObject
                                    || (((Atom) atm).getTau() instanceof Item.EnumItem
                                            && Arrays.asList(((Item.EnumItem) ((Atom) atm).getTau()).getVals())
                                                    .contains(userObject))))
                            .forEach(atom -> add(new StateNode(atom.getType().getName(), atom)));
                } else {
                    ((Item) userObject).getExprs().entrySet()
                            .forEach(expr -> add(new StateNode(expr.getKey(), expr.getValue())));
                    ((Item) userObject).getType().getPredicates().values().stream()
                            .flatMap(pred -> pred.getInstances().stream())
                            .filter(atm -> (((Atom) atm).getTau() == userObject
                                    || (((Atom) atm).getTau() instanceof Item.EnumItem
                                            && Arrays.asList(((Item.EnumItem) ((Atom) atm).getTau()).getVals())
                                                    .contains(userObject))))
                            .forEach(atom -> add(new StateNode(atom.getType().getName(), atom)));
                }
                hasLoadedChildren = true;
            }
        }
    }
}