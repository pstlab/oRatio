package it.cnr.istc.oratio.gui.timelines;

import java.awt.Color;
import java.awt.Font;
import java.text.FieldPosition;
import java.text.NumberFormat;
import java.text.ParsePosition;
import java.util.Arrays;
import java.util.Collection;
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

import it.cnr.istc.oratio.gui.TimelinesJInternalFrame.TimelineVisualizer;
import it.cnr.istc.oratio.timelines.StateVariable;
import it.cnr.istc.oratio.timelines.Timeline;
import it.cnr.istc.oratio.timelines.StateVariable.SVValue;

/**
 * StateVariableVisualizer
 */
public class StateVariableVisualizer implements TimelineVisualizer {

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
        return StateVariable.class;
    }

    @Override
    public Collection<XYPlot> getPlots(Timeline<?> timeline) {
        assert timeline instanceof StateVariable;
        StateVariable sv = (StateVariable) timeline;

        XYIntervalSeriesCollection collection = new XYIntervalSeriesCollection();
        SVValueXYIntervalSeries undefined = new SVValueXYIntervalSeries("Undefined");
        SVValueXYIntervalSeries conflicts = new SVValueXYIntervalSeries("Conflicts");
        SVValueXYIntervalSeries sv_values = new SVValueXYIntervalSeries("Values");

        for (SVValue val : sv.getValues()) {
            switch (val.getAtoms().size()) {
            case 0:
                undefined.add(val.getFrom().doubleValue(), val.getFrom().doubleValue(), val.getTo().doubleValue(), 0, 0,
                        1, val);
                break;
            case 1:
                sv_values.add(val.getFrom().doubleValue(), val.getFrom().doubleValue(), val.getTo().doubleValue(), 0, 0,
                        1, val);
                break;
            default:
                conflicts.add(val.getFrom().doubleValue(), val.getFrom().doubleValue(), val.getTo().doubleValue(), 0, 0,
                        1, val);
                break;
            }
        }

        collection.addSeries(sv_values);
        collection.addSeries(conflicts);
        collection.addSeries(undefined);

        XYBarRenderer renderer = new XYBarRenderer();
        renderer.setSeriesPaint(0, new Color(100, 250, 100));
        renderer.setSeriesPaint(1, Color.pink);
        renderer.setSeriesPaint(2, Color.lightGray);
        renderer.setBarPainter(new ReverseGradientXYBarPainter());
        renderer.setDrawBarOutline(true);
        renderer.setShadowXOffset(2);
        renderer.setShadowYOffset(2);
        renderer.setUseYInterval(true);

        Font font = new Font("SansSerif", Font.PLAIN, 9);
        XYItemLabelGenerator generator = (XYDataset dataset, int series,
                int item) -> ((SVValueXYIntervalDataItem) ((XYIntervalSeriesCollection) dataset).getSeries(series)
                        .getDataItem(item)).value.getAtoms().stream().map(atm -> atm.toString())
                                .collect(Collectors.joining(", "));
        ItemLabelPosition itLabPos = new ItemLabelPosition(ItemLabelAnchor.CENTER, TextAnchor.CENTER);
        for (int i = 0; i < collection.getSeriesCount(); i++) {
            renderer.setSeriesItemLabelGenerator(i, generator);
            renderer.setSeriesItemLabelsVisible(i, true);
            renderer.setSeriesItemLabelPaint(i, Color.black);
            renderer.setSeriesItemLabelFont(i, font);
            renderer.setSeriesPositiveItemLabelPosition(i, itLabPos);
            renderer.setSeriesToolTipGenerator(i,
                    (XYDataset dataset, int series,
                            int item) -> ((SVValueXYIntervalDataItem) ((XYIntervalSeriesCollection) dataset)
                                    .getSeries(series).getDataItem(item)).value.getAtoms().stream()
                                            .map(atm -> atm.toString()).collect(Collectors.joining(", ")));
        }

        NumberAxis na = new NumberAxis(sv.getName());
        na.setNumberFormatOverride(NF);
        na.setLabelAngle(Math.PI / 2);
        XYPlot plot = new XYPlot(collection, null, na, renderer);
        plot.setDatasetRenderingOrder(DatasetRenderingOrder.FORWARD);

        return Arrays.asList(plot);
    }

    @SuppressWarnings("serial")
    private static class SVValueXYIntervalSeries extends XYIntervalSeries {

        private SVValueXYIntervalSeries(Comparable<?> key) {
            super(key);
        }

        public void add(double x, double xLow, double xHigh, double y, double yLow, double yHigh, SVValue value) {
            super.add(new SVValueXYIntervalDataItem(x, xLow, xHigh, y, yLow, yHigh, value), true);
        }
    }

    @SuppressWarnings("serial")
    private static class SVValueXYIntervalDataItem extends XYIntervalDataItem {

        private final SVValue value;

        private SVValueXYIntervalDataItem(double x, double xLow, double xHigh, double y, double yLow, double yHigh,
                SVValue value) {
            super(x, xLow, xHigh, y, yLow, yHigh);
            this.value = value;
        }
    }
}