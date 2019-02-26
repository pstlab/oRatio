package it.cnr.istc.timelines;

import java.util.Collection;

import org.jfree.chart.plot.XYPlot;

import it.cnr.istc.oratio.timelines.Timeline;

/**
 * TimelineVisualizer
 */
public interface TimelineVisualizer {

    public Collection<XYPlot> getPlots(Timeline<?> timeline);
}