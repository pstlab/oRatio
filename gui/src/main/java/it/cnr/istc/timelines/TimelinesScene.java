package it.cnr.istc.timelines;

import org.jfree.chart.JFreeChart;
import org.jfree.chart.fx.ChartViewer;
import org.jfree.chart.plot.XYPlot;

import javafx.scene.Scene;

/**
 * TimelinesScene
 */
public class TimelinesScene extends Scene {

    private static final JFreeChart CHART = new JFreeChart(new XYPlot());

    public TimelinesScene() {
        super(new ChartViewer(CHART));
    }
}