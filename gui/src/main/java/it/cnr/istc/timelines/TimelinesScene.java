package it.cnr.istc.timelines;

import java.awt.Font;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Queue;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.DateAxis;
import org.jfree.chart.fx.ChartViewer;
import org.jfree.chart.plot.CombinedDomainXYPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;

import it.cnr.istc.riddle.Atom;
import it.cnr.istc.riddle.Core;
import it.cnr.istc.riddle.Item;
import it.cnr.istc.riddle.Predicate;
import it.cnr.istc.riddle.Type;
import javafx.scene.Scene;

/**
 * TimelinesScene
 */
public class TimelinesScene extends Scene implements TimelinesListener {

    private static final Logger LOG = Logger.getLogger(TimelinesScene.class.getName());
    private static final JFreeChart CHART = new JFreeChart(new XYPlot());
    private final Map<Type, TimelineVisualizer> timeline_visualizers = new IdentityHashMap<>();
    private final Core core;

    public TimelinesScene(final Core core) {
        super(new ChartViewer(CHART));
        this.core = core;
        try {
            timeline_visualizers.put(core.getType("StateVariable"), new StateVariableVisualizer(core));
            timeline_visualizers.put(core.getType("ReusableResource"), new ReusableResourceVisualizer(core));
            timeline_visualizers.put(core.getType("PropositionalAgent"), new PropositionalAgentVisualizer(core));
            timeline_visualizers.put(core.getType("PropositionalState"), new PropositionalStateVisualizer(core));
        } catch (ClassNotFoundException ex) {
            LOG.log(Level.SEVERE, null, ex);
        }
        timelinesChanged(core);
    }

    @Override
    public void timelinesChanged(Core core) {
        final CombinedDomainXYPlot combined_plot = new CombinedDomainXYPlot(new DateAxis("Time"));
        combined_plot.setGap(3.0);
        combined_plot.setOrientation(PlotOrientation.VERTICAL);

        Map<Item, Collection<Atom>> atoms = new IdentityHashMap<>();
        for (Type t : core.getTypes().values()) {
            if (getTimeline(t) != null) {
                t.getInstances().forEach(i -> atoms.put(i, new ArrayList<>()));
                for (Predicate p : t.getPredicates().values())
                    p.getInstances().stream().map(atm -> (Atom) atm)
                            .filter(atm -> (atm.getState() == Atom.AtomState.Active)).forEach(atm -> {
                                Item tau = atm.getTau();
                                if (tau instanceof Item.EnumItem)
                                    for (Item val : ((Item.EnumItem) tau).getVals())
                                        atoms.get(val).add(atm);
                                else
                                    atoms.get(tau).add(atm);
                            });
            }
        }

        for (Map.Entry<String, Item> entry : core.getExprs().entrySet()) {
            Type timeline = getTimeline(entry.getValue().getType());
            if (timeline != null)
                for (XYPlot plot : timeline_visualizers.get(timeline).getPlots(entry.getValue(),
                        atoms.get(entry.getValue())))
                    combined_plot.add(plot);
        }

        ((ChartViewer) getRoot())
                .setChart(new JFreeChart("", new Font("SansSerif", Font.BOLD, 14), combined_plot, false));
    }

    private Type getTimeline(Type t) {
        Queue<Type> q = new ArrayDeque<>();
        q.add(t);
        while (!q.isEmpty()) {
            Type c_type = q.poll();
            if (timeline_visualizers.containsKey(c_type))
                return c_type;
            q.addAll(c_type.getSuperclasses());
        }
        return null;
    }
}