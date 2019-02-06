package it.cnr.istc.timelines;

import java.awt.Color;
import java.awt.Font;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

import org.jfree.chart.annotations.XYTextAnnotation;
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
    public Collection<XYPlot> getPlots(Item itm, Collection<Atom> atoms) {
        XYIntervalSeriesCollection collection = new XYIntervalSeriesCollection();

        ActionValueXYIntervalSeries actions = new ActionValueXYIntervalSeries("Actions");

        ArrayList<Double> ends = new ArrayList<>(10);
        ends.add(0d);
        for (Atom atom : atoms) {
            double start_pulse, end_pulse;
            if (atom.getType().getSuperclasses().stream().filter(t -> t.getName().equals("ImpulsivePredicate"))
                    .findAny().isPresent()) {
                start_pulse = ((Item.ArithItem) atom.getExpr("at")).getValue().doubleValue();
                end_pulse = start_pulse + 1;
            } else {
                start_pulse = ((Item.ArithItem) atom.getExpr("start")).getValue().doubleValue();
                end_pulse = ((Item.ArithItem) atom.getExpr("end")).getValue().doubleValue();
            }
            double y = getYValue(start_pulse, end_pulse, ends);
            actions.add(start_pulse, start_pulse, end_pulse, y - 1, y - 1, y, atom);
        }
        collection.addSeries(actions);

        XYBarRenderer renderer = new XYBarRenderer();
        renderer.setSeriesPaint(0, Color.lightGray);
        renderer.setBarPainter(new ReverseGradientXYBarPainter());
        renderer.setDrawBarOutline(true);
        renderer.setShadowXOffset(2);
        renderer.setShadowYOffset(2);
        renderer.setUseYInterval(true);

        Font font = new Font("SansSerif", Font.PLAIN, 9);
        XYItemLabelGenerator generator = (XYDataset dataset, int series,
                int item) -> ((ActionValueXYIntervalDataItem) ((XYIntervalSeriesCollection) dataset).getSeries(series)
                        .getDataItem(item)).atom.toString();
        ItemLabelPosition itLabPos = new ItemLabelPosition(ItemLabelAnchor.CENTER, TextAnchor.CENTER);
        for (int i = 0; i < collection.getSeriesCount(); i++) {
            renderer.setSeriesItemLabelGenerator(i, generator);
            renderer.setSeriesItemLabelsVisible(i, true);
            renderer.setSeriesItemLabelPaint(i, Color.black);
            renderer.setSeriesItemLabelFont(i, font);
            renderer.setSeriesPositiveItemLabelPosition(i, itLabPos);
            renderer.setSeriesToolTipGenerator(i,
                    (XYDataset dataset, int series,
                            int item) -> ((ActionValueXYIntervalDataItem) ((XYIntervalSeriesCollection) dataset)
                                    .getSeries(series).getDataItem(item)).atom.toString());
        }

        XYPlot plot = new XYPlot(collection, null, new NumberAxis(""), renderer);
        plot.getRangeAxis().setVisible(false);
        plot.setDatasetRenderingOrder(DatasetRenderingOrder.FORWARD);
        XYTextAnnotation annotation = new XYTextAnnotation(core.guessName(itm), 0, 1);
        annotation.setTextAnchor(TextAnchor.TOP_LEFT);
        plot.addAnnotation(annotation);

        return Arrays.asList(plot);
    }

    @SuppressWarnings("serial")
    private static class ActionValueXYIntervalSeries extends XYIntervalSeries {

        private ActionValueXYIntervalSeries(Comparable<?> key) {
            super(key);
        }

        public void add(double x, double xLow, double xHigh, double y, double yLow, double yHigh, Atom atom) {
            super.add(new ActionValueXYIntervalDataItem(x, xLow, xHigh, y, yLow, yHigh, atom), true);
        }
    }

    @SuppressWarnings("serial")
    private static class ActionValueXYIntervalDataItem extends XYIntervalDataItem {

        private final Atom atom;

        private ActionValueXYIntervalDataItem(double x, double xLow, double xHigh, double y, double yLow, double yHigh,
                Atom atom) {
            super(x, xLow, xHigh, y, yLow, yHigh);
            this.atom = atom;
        }
    }

    private static int getYValue(double start, double end, ArrayList<Double> ends) {
        for (int i = 0; i < ends.size(); i++)
            if (ends.get(i) <= start) {
                ends.set(i, end);
                return (i * 2);
            }
        ends.add(end);
        return ((ends.size() - 1) * 2);
    }
}