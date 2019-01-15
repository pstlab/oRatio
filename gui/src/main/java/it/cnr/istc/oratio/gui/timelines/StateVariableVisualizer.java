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
import java.awt.Color;
import java.awt.Font;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.labels.ItemLabelAnchor;
import org.jfree.chart.labels.ItemLabelPosition;
import org.jfree.chart.labels.XYItemLabelGenerator;
import org.jfree.chart.plot.DatasetRenderingOrder;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYBarRenderer;
import org.jfree.chart.ui.TextAnchor;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYIntervalDataItem;
import org.jfree.data.xy.XYIntervalSeries;
import org.jfree.data.xy.XYIntervalSeriesCollection;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class StateVariableVisualizer implements TimelineVisualizer {

    private final Core core;

    public StateVariableVisualizer(Core core) {
        this.core = core;
    }

    @Override
    public XYPlot getPlot(Item itm, Collection<Atom> atoms) {
        XYIntervalSeriesCollection collection = new XYIntervalSeriesCollection();
        SVValueXYIntervalSeries undefined = new SVValueXYIntervalSeries("Undefined");
        SVValueXYIntervalSeries sv_values = new SVValueXYIntervalSeries("Values");
        SVValueXYIntervalSeries conflicts = new SVValueXYIntervalSeries("Conflicts");

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
        for (int i = 1; i < c_pulses_array.length; i++) {
            switch (overlapping_formulas.size()) {
                case 0:
                    undefined.add(c_pulses_array[i - 1], c_pulses_array[i - 1], c_pulses_array[i], 0, 0, 1, new SVValue(overlapping_formulas));
                    break;
                case 1:
                    sv_values.add(c_pulses_array[i - 1], c_pulses_array[i - 1], c_pulses_array[i], 0, 0, 1, new SVValue(overlapping_formulas));
                    break;
                default:
                    conflicts.add(c_pulses_array[i - 1], c_pulses_array[i - 1], c_pulses_array[i], 0, 0, 1, new SVValue(overlapping_formulas));
                    break;
            }
            if (starting_values.containsKey(c_pulses_array[i])) {
                overlapping_formulas.addAll(starting_values.get(c_pulses_array[i]));
            }
            if (ending_values.containsKey(c_pulses_array[i])) {
                overlapping_formulas.removeAll(ending_values.get(c_pulses_array[i]));
            }
        }
        collection.addSeries(undefined);
        collection.addSeries(sv_values);
        collection.addSeries(conflicts);

        XYBarRenderer renderer = new XYBarRenderer();
        renderer.setSeriesPaint(0, Color.lightGray);
        renderer.setSeriesPaint(1, new Color(100, 250, 100));
        renderer.setSeriesPaint(2, Color.pink);
        renderer.setBarPainter(new ReverseGradientXYBarPainter());
        renderer.setDrawBarOutline(true);
        renderer.setShadowXOffset(2);
        renderer.setShadowYOffset(2);
        renderer.setUseYInterval(true);

//        renderer.setBaseItemLabelsVisible(true);
//        renderer.setBaseItemLabelPaint(Color.black);
        Font font = new Font("SansSerif", Font.PLAIN, 9);
//        renderer.setBaseItemLabelFont(font);
        XYItemLabelGenerator generator = (XYDataset dataset, int series, int item) -> ((SVValueXYIntervalDataItem) ((XYIntervalSeriesCollection) dataset).getSeries(series).getDataItem(item)).value.toString();
        ItemLabelPosition itLabPos = new ItemLabelPosition(ItemLabelAnchor.CENTER, TextAnchor.CENTER);
//        renderer.setBasePositiveItemLabelPosition(itLabPos);
        for (int i = 0; i < collection.getSeriesCount(); i++) {
            renderer.setSeriesItemLabelGenerator(i, generator);
            renderer.setSeriesItemLabelsVisible(i, true);
            renderer.setSeriesItemLabelPaint(i, Color.black);
            renderer.setSeriesItemLabelFont(i, font);
            renderer.setSeriesPositiveItemLabelPosition(i, itLabPos);
            renderer.setSeriesToolTipGenerator(i, (XYDataset dataset, int series, int item) -> ((SVValueXYIntervalDataItem) ((XYIntervalSeriesCollection) dataset).getSeries(series).getDataItem(item)).value.toString());
        }

        XYPlot plot = new XYPlot(collection, null, new NumberAxis(""), renderer);
        plot.getRangeAxis().setVisible(false);
        plot.setDatasetRenderingOrder(DatasetRenderingOrder.FORWARD);

        return plot;
    }

    private static class SVValue {

        final List<Atom> atoms;

        SVValue(List<Atom> atoms) {
            this.atoms = new ArrayList<>(atoms);
        }

        private String toString(Atom atom) {
            StringBuilder sb = new StringBuilder();
            sb.append(atom.getType().getName()).append("(");
            for (Map.Entry<String, Item> expr : atom.getExprs().entrySet()) {
                sb.append(", ");
                switch (expr.getValue().getType().getName()) {
                    case Core.BOOL:
                        sb.append(expr.getKey()).append(" = ").append(((Item.BoolItem) expr.getValue()).getValue());
                        break;
                    case Core.INT:
                    case Core.REAL:
                        sb.append(expr.getKey()).append(" = ").append(((Item.ArithItem) expr.getValue()).getValue());
                        break;
                    case Core.STRING:
                        sb.append(expr.getKey()).append(" = ").append(((Item.StringItem) expr.getValue()).getValue());
                        break;
                    default:
                        sb.append(expr.getKey());
                }
            }
            sb.append(")");
            return sb.toString().replace("(, ", "(");
        }

        @Override
        public String toString() {
            switch (atoms.size()) {
                case 0:
                    return "";
                case 1:
                    return toString(atoms.get(0));
                default:
                    return "{" + atoms.stream().map(atom -> toString(atom)).collect(Collectors.joining(", ")) + "}";
            }
        }
    }

    @SuppressWarnings({"serial", "CloneableImplementsClone"})
    private static class SVValueXYIntervalSeries extends XYIntervalSeries {

        private SVValueXYIntervalSeries(Comparable<?> key) {
            super(key);
        }

        public void add(double x, double xLow, double xHigh, double y, double yLow, double yHigh, SVValue value) {
            super.add(new SVValueXYIntervalDataItem(x, xLow, xHigh, y, yLow, yHigh, value), true);
        }
    }

    @SuppressWarnings({"serial", "CloneableImplementsClone"})
    private static class SVValueXYIntervalDataItem extends XYIntervalDataItem {

        private final SVValue value;

        private SVValueXYIntervalDataItem(double x, double xLow, double xHigh, double y, double yLow, double yHigh, SVValue value) {
            super(x, xLow, xHigh, y, yLow, yHigh);
            this.value = value;
        }
    }
}
