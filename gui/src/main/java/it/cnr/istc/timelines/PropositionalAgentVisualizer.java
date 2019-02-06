package it.cnr.istc.timelines;

import java.awt.Color;
import java.awt.Font;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;

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

import it.cnr.istc.riddle.Atom;
import it.cnr.istc.riddle.Core;
import it.cnr.istc.riddle.Item;

/**
 * PropositionalAgentVisualizer
 */
public class PropositionalAgentVisualizer implements TimelineVisualizer {

    private final Core core;

    PropositionalAgentVisualizer(Core core) {
        this.core = core;
    }

    @Override
    public XYPlot getPlot(Item itm, Collection<Atom> atoms) {
        XYIntervalSeriesCollection collection = new XYIntervalSeriesCollection();

        ActionValueXYIntervalSeries actions = new ActionValueXYIntervalSeries("Actions");

        ArrayList<Double> ends = new ArrayList<>(10);
        ends.add(0d);
        for (Atom atom : atoms) {
            double start_pulse, end_pulse;
            if (atom.getType().getSuperclasses().stream().filter(t -> t.getName().equals("ImpulsivePredicate"))
                    .findAny().isPresent()) {
                start_pulse = ((Item.ArithItem) atom.getExpr("at")).getValue().doubleValue();
                end_pulse = start_pulse + 0.5;
            } else {
                start_pulse = ((Item.ArithItem) atom.getExpr("start")).getValue().doubleValue();
                end_pulse = ((Item.ArithItem) atom.getExpr("end")).getValue().doubleValue();
            }
            double y = getYValue(start_pulse, end_pulse, ends);
            actions.add(start_pulse, start_pulse, end_pulse, y, y - 1, y, new Action(atom));
        }
        collection.addSeries(actions);

        XYBarRenderer renderer = new XYBarRenderer();
        renderer.setSeriesPaint(0, Color.lightGray);
        renderer.setSeriesPaint(1, new Color(100, 250, 100));
        renderer.setSeriesPaint(2, Color.pink);
        renderer.setBarPainter(new ReverseGradientXYBarPainter());
        renderer.setDrawBarOutline(true);
        renderer.setShadowXOffset(2);
        renderer.setShadowYOffset(2);
        renderer.setUseYInterval(true);

        // renderer.setBaseItemLabelsVisible(true);
        // renderer.setBaseItemLabelPaint(Color.black);
        Font font = new Font("SansSerif", Font.PLAIN, 9);
        // renderer.setBaseItemLabelFont(font);
        XYItemLabelGenerator generator = (XYDataset dataset, int series,
                int item) -> ((ActionValueXYIntervalDataItem) ((XYIntervalSeriesCollection) dataset).getSeries(series)
                        .getDataItem(item)).action.toString();
        ItemLabelPosition itLabPos = new ItemLabelPosition(ItemLabelAnchor.CENTER, TextAnchor.CENTER);
        // renderer.setBasePositiveItemLabelPosition(itLabPos);
        for (int i = 0; i < collection.getSeriesCount(); i++) {
            renderer.setSeriesItemLabelGenerator(i, generator);
            renderer.setSeriesItemLabelsVisible(i, true);
            renderer.setSeriesItemLabelPaint(i, Color.black);
            renderer.setSeriesItemLabelFont(i, font);
            renderer.setSeriesPositiveItemLabelPosition(i, itLabPos);
            renderer.setSeriesToolTipGenerator(i,
                    (XYDataset dataset, int series,
                            int item) -> ((ActionValueXYIntervalDataItem) ((XYIntervalSeriesCollection) dataset)
                                    .getSeries(series).getDataItem(item)).action.toString());
        }

        XYPlot plot = new XYPlot(collection, null, new NumberAxis(""), renderer);
        plot.getRangeAxis().setVisible(false);
        plot.setDatasetRenderingOrder(DatasetRenderingOrder.FORWARD);

        return plot;
    }

    private static class Action {

        final Atom atom;

        Action(Atom atom) {
            this.atom = atom;
        }

        @Override
        public String toString() {
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
    }

    @SuppressWarnings("serial")
    private static class ActionValueXYIntervalSeries extends XYIntervalSeries {

        private ActionValueXYIntervalSeries(Comparable<?> key) {
            super(key);
        }

        public void add(double x, double xLow, double xHigh, double y, double yLow, double yHigh, Action action) {
            super.add(new ActionValueXYIntervalDataItem(x, xLow, xHigh, y, yLow, yHigh, action), true);
        }
    }

    @SuppressWarnings("serial")
    private static class ActionValueXYIntervalDataItem extends XYIntervalDataItem {

        private final Action action;

        private ActionValueXYIntervalDataItem(double x, double xLow, double xHigh, double y, double yLow, double yHigh,
                Action action) {
            super(x, xLow, xHigh, y, yLow, yHigh);
            this.action = action;
        }
    }

    private static int getYValue(double start, double end, ArrayList<Double> ends) {
        for (int i = 0; i < ends.size(); i++)
            if (ends.get(i) <= start) {
                ends.set(i, end);
                return (i * 2) + 1;
            }
        ends.add(end);
        return ((ends.size() - 1) * 2) + 1;
    }
}