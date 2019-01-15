/*
 * Copyright (C) 2019 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
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
package it.cnr.istc.oratio.gui.timelines;

import it.cnr.istc.oratio.gui.riddle.Atom;
import it.cnr.istc.oratio.gui.riddle.Core;
import it.cnr.istc.oratio.gui.riddle.Item;
import it.cnr.istc.oratio.gui.riddle.Predicate;
import it.cnr.istc.oratio.gui.riddle.Type;
import java.awt.Cursor;
import java.awt.Font;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Queue;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.DateAxis;
import org.jfree.chart.plot.CombinedDomainXYPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class TimelinesChart extends ChartPanel {

    private static final Logger LOG = Logger.getLogger(TimelinesChart.class.getName());
    private static final JFreeChart CHART = new JFreeChart(new XYPlot());
    private static final Cursor DEFAULT_CURSOR = new Cursor(Cursor.DEFAULT_CURSOR);
    private static final Cursor HAND_CURSOR = new Cursor(Cursor.HAND_CURSOR);
    private final Map<Type, TimelineVisualizer> timeline_visualizers = new LinkedHashMap<>();
    private Core core;

    public TimelinesChart() {
        super(CHART);
    }

    public void setCore(Core core) {
        if (this.core == null) {
            try {
                timeline_visualizers.put(core.getType("StateVariable"), new StateVariableVisualizer(core));
                timeline_visualizers.put(core.getType("ReusableResource"), new ReusableResourceVisualizer(core));
            } catch (ClassNotFoundException ex) {
                LOG.log(Level.SEVERE, null, ex);
            }
        }
        this.core = core;
        final CombinedDomainXYPlot combined_plot = new CombinedDomainXYPlot(new DateAxis("Time"));
        combined_plot.setGap(3.0);
        combined_plot.setOrientation(PlotOrientation.VERTICAL);

        Map<Item, Collection<Atom>> atoms = new IdentityHashMap<>();
        for (Type t : core.getTypes().values()) {
            if (isTimeline(t)) {
                t.getInstances().forEach(i -> atoms.put(i, new ArrayList<>()));
                for (Predicate p : t.getPredicates().values()) {
                    p.getInstances().stream().map(atm -> (Atom) atm).filter(atm -> (atm.getState() == Atom.AtomState.Active)).forEach(atm -> {
                        Item tau = atm.getTau();
                        if (tau instanceof Item.EnumItem) {
                            for (Item val : ((Item.EnumItem) tau).getVals()) {
                                atoms.get(val).add(atm);
                            }
                        } else {
                            atoms.get(tau).add(atm);
                        }
                    });
                }
            }
        }

        for (Map.Entry<String, Item> entry : core.getExprs().entrySet()) {
            if (isTimeline(entry.getValue().getType())) {
                combined_plot.add(timeline_visualizers.get(entry.getValue().getType()).getPlot(entry.getValue(), atoms.get(entry.getValue())));
            }
        }
        setChart(new JFreeChart("", new Font("SansSerif", Font.BOLD, 14), combined_plot, false));
    }

    private boolean isTimeline(Type t) {
        Queue<Type> q = new ArrayDeque<>();
        q.add(t);
        while (!q.isEmpty()) {
            Type c_type = q.poll();
            if (timeline_visualizers.containsKey(c_type)) {
                return true;
            }
            q.addAll(c_type.getSuperclasses());
        }
        return false;
    }
}
