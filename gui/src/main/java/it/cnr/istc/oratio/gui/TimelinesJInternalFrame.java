package it.cnr.istc.oratio.gui;

import java.awt.Font;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.Map;

import javax.swing.JInternalFrame;
import javax.swing.SwingUtilities;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.DateAxis;
import org.jfree.chart.plot.CombinedDomainXYPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;

import it.cnr.istc.oratio.Context;
import it.cnr.istc.oratio.StateListener;
import it.cnr.istc.oratio.riddle.Core;
import it.cnr.istc.oratio.timelines.Timeline;

/**
 * TimelinesJInternalFrame
 */
public class TimelinesJInternalFrame extends JInternalFrame implements StateListener {

    private static final JFreeChart CHART = new JFreeChart(new XYPlot());
    private final Core core;
    private final ChartPanel panel = new ChartPanel(CHART);

    private final Map<Class<? extends Timeline<?>>, TimelineVisualizer> visualizers = new IdentityHashMap<>();

    public TimelinesJInternalFrame(final Core core) {
        super("Timelines");
        this.core = core;

        add(panel);

        setIconifiable(true);
        setMaximizable(true);
        setResizable(true);
        pack();
    }

    public void addVisualizer(TimelineVisualizer vis) {
        visualizers.put(vis.getType(), vis);
    }

    @Override
    public void log(String log) {
    }

    @Override
    public void stateChanged(Core core) {
        SwingUtilities.invokeLater(() -> {
            final CombinedDomainXYPlot combined_plot = new CombinedDomainXYPlot(new DateAxis("Time"));
            combined_plot.setGap(3.0);
            combined_plot.setOrientation(PlotOrientation.VERTICAL);
            for (Timeline<?> tl : Context.getContext().getTimelines()) {
                TimelineVisualizer vis = visualizers.get(tl.getClass());
                if (vis != null)
                    for (XYPlot plot : vis.getPlots(tl))
                        combined_plot.add(plot);
            }
            panel.setChart(new JFreeChart("", new Font("SansSerif", Font.BOLD, 14), combined_plot, false));
        });
    }

    public interface TimelineVisualizer {

        public Class<? extends Timeline<?>> getType();

        public Collection<XYPlot> getPlots(Timeline<?> timeline);
    }
}