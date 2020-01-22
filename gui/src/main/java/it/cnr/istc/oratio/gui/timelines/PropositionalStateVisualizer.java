package it.cnr.istc.oratio.gui.timelines;

import java.awt.Color;
import java.text.FieldPosition;
import java.text.NumberFormat;
import java.text.ParsePosition;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.DatasetRenderingOrder;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYBarRenderer;
import org.jfree.data.xy.XYIntervalSeries;
import org.jfree.data.xy.XYIntervalSeriesCollection;

import it.cnr.istc.oratio.gui.TimelinesJInternalFrame.TimelineVisualizer;
import it.cnr.istc.oratio.timelines.PropositionalState;
import it.cnr.istc.oratio.timelines.PropositionalState.Fluent;
import it.cnr.istc.oratio.timelines.PropositionalState.Literal;
import it.cnr.istc.oratio.timelines.Timeline;

/**
 * PropositionalStateVisualizer
 */
public class PropositionalStateVisualizer implements TimelineVisualizer {

    private static final NumberFormat NF = new NumberFormat() {

        private static final long serialVersionUID = 1L;

        @Override
        public StringBuffer format(final double number, final StringBuffer toAppendTo, final FieldPosition pos) {
            return toAppendTo;
        }

        @Override
        public StringBuffer format(final long number, final StringBuffer toAppendTo, final FieldPosition pos) {
            return toAppendTo;
        }

        @Override
        public Number parse(final String source, final ParsePosition parsePosition) {
            return 0;
        }
    };

    @Override
    public Class<? extends Timeline<?>> getType() {
        return PropositionalState.class;
    }

    @Override
    public Collection<XYPlot> getPlots(final Timeline<?> timeline) {
        assert timeline instanceof PropositionalState;
        final PropositionalState ps = (PropositionalState) timeline;

        final XYBarRenderer renderer = new XYBarRenderer();
        renderer.setSeriesPaint(0, new Color(100, 250, 100));
        renderer.setSeriesPaint(1, Color.lightGray);
        renderer.setBarPainter(new ReverseGradientXYBarPainter());
        renderer.setDrawBarOutline(true);
        renderer.setShadowXOffset(2);
        renderer.setShadowYOffset(2);
        renderer.setUseYInterval(true);

        final List<XYPlot> plots = new ArrayList<>();
        for (final Fluent fluent : ps.getValues()) {
            final XYIntervalSeriesCollection collection = new XYIntervalSeriesCollection();
            final XYIntervalSeries true_series = new XYIntervalSeries("True");
            final XYIntervalSeries false_series = new XYIntervalSeries("False");
            collection.addSeries(true_series);
            collection.addSeries(false_series);

            for (final Literal lit : fluent.getValues()) {
                switch (lit.getPolarity()) {
                case True:
                    true_series.add(lit.getFrom().doubleValue(), lit.getFrom().doubleValue(), lit.getTo().doubleValue(),
                            0, 0, 1);
                    break;
                case False:
                    false_series.add(lit.getFrom().doubleValue(), lit.getFrom().doubleValue(),
                            lit.getTo().doubleValue(), 0, 0, 1);
                    break;
                default:
                    break;
                }
            }

            final NumberAxis na = new NumberAxis(fluent.getName());
            na.setNumberFormatOverride(NF);
            na.setLabelAngle(Math.PI / 2);
            final XYPlot plot = new XYPlot(collection, null, na, renderer);
            plot.setDatasetRenderingOrder(DatasetRenderingOrder.FORWARD);

            plots.add(plot);
        }

        Collections.sort(plots, (final XYPlot p1, final XYPlot p2) -> p1.getRangeAxis().getLabel()
                .compareTo(p2.getRangeAxis().getLabel()));
        return plots;
    }
}