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

import it.cnr.istc.oratio.gui.riddle.Core;
import it.cnr.istc.oratio.gui.riddle.Item;
import javax.swing.tree.DefaultMutableTreeNode;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class StateTreeNode extends DefaultMutableTreeNode {

    final String name;
    private boolean hasLoadedChildren = false;

    public StateTreeNode(final String name, final Item item) {
        super(item);
        this.name = name;
    }

    public boolean hasLoadedChildren() {
        return hasLoadedChildren;
    }

    @Override
    public boolean isLeaf() {
        if (userObject == null) {
            return false;
        } else {
            switch (((Item) userObject).getType().getName()) {
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
    public boolean getAllowsChildren() {
        return !((Item) userObject).getExprs().isEmpty();
    }

    public void loadChildren() {
        if (!hasLoadedChildren) {
            hasLoadedChildren = true;
            ((Item) userObject).getExprs().entrySet().forEach(xpr -> add(new StateTreeNode(xpr.getKey(), xpr.getValue())));
        }
    }
}
