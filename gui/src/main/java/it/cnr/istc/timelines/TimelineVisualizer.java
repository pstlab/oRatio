package it.cnr.istc.timelines;

import java.util.Collection;

import org.jfree.chart.plot.XYPlot;

import it.cnr.istc.riddle.Atom;
import it.cnr.istc.riddle.Item;

/**
 * TimelineVisualizer
 */
public interface TimelineVisualizer {

    public Collection<XYPlot> getPlots(Item itm, Collection<Atom> atoms);
}