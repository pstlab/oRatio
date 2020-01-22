package it.cnr.istc.oratio.gui.timelines;

import java.awt.Color;
import java.awt.Font;
import java.text.FieldPosition;
import java.text.NumberFormat;
import java.text.ParsePosition;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

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

import it.cnr.istc.oratio.gui.TimelinesJInternalFrame.TimelineVisualizer;
import it.cnr.istc.oratio.timelines.PropositionalAgent;
import it.cnr.istc.oratio.timelines.Timeline;
import it.cnr.istc.oratio.timelines.PropositionalAgent.Action;

/**
 * PropositionalAgentVisualizer
 */
public class PropositionalAgentVisualizer implements TimelineVisualizer {

    private static final NumberFormat NF = new NumberFormat() {

        private static final long serialVersionUID = 1L;

        @Override
        public StringBuffer format(double number, StringBuffer toAppendTo, FieldPosition pos) {
            return toAppendTo;
        }

        @Override
        public StringBuffer format(long number, StringBuffer toAppendTo, FieldPosition pos) {
            return toAppendTo;
        }

        @Override
        public Number parse(String source, ParsePosition parsePosition) {
            return 0;
        }
    };

    @Override
    public Class<? extends Timeline<?>> getType() {
        return PropositionalAgent.class;
    }

    @Override
    public Collection<XYPlot> getPlots(Timeline<?> timeline) {
        assert timeline instanceof PropositionalAgent;
        PropositionalAgent pa = (PropositionalAgent) timeline;

        XYIntervalSeriesCollection collection = new XYIntervalSeriesCollection();

        ActionValueXYIntervalSeries actions = new ActionValueXYIntervalSeries("Actions");

        ArrayList<Double> ends = new ArrayList<>(10);
        ends.add(0d);
        for (Action action : pa.getValues()) {
            double start_pulse, end_pulse;
            if (action.isImpulsive()) {
                start_pulse = action.getFrom().doubleValue();
                end_pulse = start_pulse + 1;
            } else {
                start_pulse = action.getFrom().doubleValue();
                end_pulse = action.getTo().doubleValue();
            }
            double y = getYValue(start_pulse, end_pulse, ends);
            actions.add(start_pulse, start_pulse, end_pulse, y, y, y + 1, action);
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
                        .getDataItem(item)).action.toString();
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
                                    .getSeries(series).getDataItem(item)).action.getAtom().toString());
        }

        NumberAxis na = new NumberAxis(pa.getName());
        na.setNumberFormatOverride(NF);
        na.setLabelAngle(Math.PI / 2);
        XYPlot plot = new XYPlot(collection, null, na, renderer);
        plot.setDatasetRenderingOrder(DatasetRenderingOrder.FORWARD);

        return Arrays.asList(plot);
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
                return (i * 2);
            }
        ends.add(end);
        return ((ends.size() - 1) * 2);
    }
}