package it.cnr.istc.timelines;

import java.awt.Font;
import java.util.IdentityHashMap;
import java.util.Map;

import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.DateAxis;
import org.jfree.chart.fx.ChartViewer;
import org.jfree.chart.plot.CombinedDomainXYPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;

import it.cnr.istc.oratio.Context;
import it.cnr.istc.oratio.StateListener;
import it.cnr.istc.oratio.riddle.Core;
import it.cnr.istc.oratio.timelines.PropositionalAgent;
import it.cnr.istc.oratio.timelines.PropositionalState;
import it.cnr.istc.oratio.timelines.ReusableResource;
import it.cnr.istc.oratio.timelines.StateVariable;
import it.cnr.istc.oratio.timelines.Timeline;
import javafx.scene.Scene;

/**
 * TimelinesScene
 */
public class TimelinesScene extends Scene implements StateListener {

    private static final JFreeChart CHART = new JFreeChart(new XYPlot());
    private final Map<Class<? extends Timeline<?>>, TimelineVisualizer> timeline_visualizers = new IdentityHashMap<>();

    public TimelinesScene(final Core core) {
        super(new ChartViewer(CHART));
        timeline_visualizers.put(StateVariable.class, new StateVariableVisualizer());
        timeline_visualizers.put(ReusableResource.class, new ReusableResourceVisualizer());
        timeline_visualizers.put(PropositionalAgent.class, new PropositionalAgentVisualizer());
        timeline_visualizers.put(PropositionalState.class, new PropositionalStateVisualizer());
        stateChanged(core);
    }

    @Override
    public void stateChanged(Core core) {
        final CombinedDomainXYPlot combined_plot = new CombinedDomainXYPlot(new DateAxis("Time"));
        combined_plot.setGap(3.0);
        combined_plot.setOrientation(PlotOrientation.VERTICAL);

        for (Timeline<?> tln : Context.getContext().getTimelines())
            for (XYPlot plot : timeline_visualizers.get(tln.getClass()).getPlots(tln))
                combined_plot.add(plot);

        ((ChartViewer) getRoot())
                .setChart(new JFreeChart("", new Font("SansSerif", Font.BOLD, 14), combined_plot, false));
    }

    @Override
    public void log(String log) {
    }
}