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

import it.cnr.istc.oratio.InfRational;
import it.cnr.istc.oratio.gui.riddle.Atom;
import it.cnr.istc.oratio.gui.riddle.Core;
import it.cnr.istc.oratio.gui.riddle.Item;
import java.awt.BasicStroke;
import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.DatasetRenderingOrder;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYStepRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class ReusableResourceVisualizer implements TimelineVisualizer {

    private final Core core;

    public ReusableResourceVisualizer(Core core) {
        this.core = core;
    }

    @Override
    public XYPlot getPlot(Item itm, Collection<Atom> atoms) {
        XYSeriesCollection collection = new XYSeriesCollection();
        XYSeries profile = new XYSeries("Profile");
        XYSeries capacity_series = new XYSeries("Capacity");

        // For each pulse the atoms starting at that pulse
        Map<Double, Collection<Atom>> starting_values = new HashMap<>(atoms.size());
        // For each pulse the atoms ending at that pulse
        Map<Double, Collection<Atom>> ending_values = new HashMap<>(atoms.size());
        // The pulses of the timeline
        Set<Double> c_pulses = new TreeSet<>();
        c_pulses.add(((Item.ArithItem) core.getExpr("origin")).getValue().doubleValue());
        c_pulses.add(((Item.ArithItem) core.getExpr("horizon")).getValue().doubleValue());

        for (Atom atom : atoms) {
            double start_pulse = ((Item.ArithItem) atom.getExpr("start")).getValue().doubleValue();
            c_pulses.add(start_pulse);
            if (!starting_values.containsKey(start_pulse)) {
                starting_values.put(start_pulse, new ArrayList<>(atoms.size()));
            }
            starting_values.get(start_pulse).add(atom);
            double end_pulse = ((Item.ArithItem) atom.getExpr("end")).getValue().doubleValue();
            c_pulses.add(end_pulse);
            if (!ending_values.containsKey(end_pulse)) {
                ending_values.put(end_pulse, new ArrayList<>(atoms.size()));
            }
            ending_values.get(end_pulse).add(atom);
        }

        Double[] c_pulses_array = c_pulses.toArray(new Double[c_pulses.size()]);
        // Push values to timeline according to pulses...
        List<Atom> overlapping_formulas = new ArrayList<>(atoms.size());
        if (starting_values.containsKey(c_pulses_array[0])) {
            overlapping_formulas.addAll(starting_values.get(c_pulses_array[0]));
        }
        if (ending_values.containsKey(c_pulses_array[0])) {
            overlapping_formulas.removeAll(ending_values.get(c_pulses_array[0]));
        }
        profile.add((double) c_pulses_array[0], 0);
        for (int i = 1; i < c_pulses_array.length; i++) {
            InfRational c_usage = new InfRational();
            overlapping_formulas.forEach(atom -> c_usage.add(((Item.ArithItem) atom.getExpr("amount")).getValue()));
            profile.add((double) c_pulses_array[i - 1], c_usage.doubleValue());
            profile.add((double) c_pulses_array[i], c_usage.doubleValue());
            if (starting_values.containsKey(c_pulses_array[i])) {
                overlapping_formulas.addAll(starting_values.get(c_pulses_array[i]));
            }
            if (ending_values.containsKey(c_pulses_array[i])) {
                overlapping_formulas.removeAll(ending_values.get(c_pulses_array[i]));
            }
        }
        profile.add((double) c_pulses_array[c_pulses_array.length - 1], 0);
        collection.addSeries(profile);

        capacity_series.add((double) c_pulses_array[0], ((Item.ArithItem) itm.getExpr("capacity")).getValue());
        capacity_series.add((double) c_pulses_array[c_pulses_array.length - 1], ((Item.ArithItem) itm.getExpr("capacity")).getValue());
        collection.addSeries(capacity_series);

        XYStepRenderer renderer = new XYStepRenderer();
        renderer.setSeriesPaint(0, Color.BLUE);
        renderer.setSeriesToolTipGenerator(0, (XYDataset dataset, int series, int item) -> ((XYSeriesCollection) dataset).getSeries(series).getDataItem(item).getY().toString());
        renderer.setSeriesPaint(1, new Color(135, 0, 0));
        renderer.setSeriesStroke(1, new BasicStroke(1.5f));

        XYPlot plot = new XYPlot(collection, null, new NumberAxis(""), renderer);
        plot.setDatasetRenderingOrder(DatasetRenderingOrder.FORWARD);

        return plot;
    }
}
