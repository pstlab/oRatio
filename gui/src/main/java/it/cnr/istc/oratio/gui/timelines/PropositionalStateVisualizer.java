package it.cnr.istc.oratio.gui.timelines;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;

import org.jfree.chart.annotations.XYTextAnnotation;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.DatasetRenderingOrder;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYBarRenderer;
import org.jfree.chart.ui.TextAnchor;
import org.jfree.data.xy.XYIntervalSeries;
import org.jfree.data.xy.XYIntervalSeriesCollection;

import it.cnr.istc.oratio.gui.TimelinesJInternalFrame.TimelineVisualizer;
import it.cnr.istc.oratio.timelines.PropositionalState;
import it.cnr.istc.oratio.timelines.Timeline;
import it.cnr.istc.oratio.timelines.PropositionalState.Fluent;
import it.cnr.istc.oratio.timelines.PropositionalState.Literal;

/**
 * PropositionalStateVisualizer
 */
public class PropositionalStateVisualizer implements TimelineVisualizer {

    @Override
    public Class<? extends Timeline<?>> getType() {
        return PropositionalState.class;
    }

    @Override
    public Collection<XYPlot> getPlots(Timeline<?> timeline) {
        assert timeline instanceof PropositionalState;
        PropositionalState ps = (PropositionalState) timeline;

        XYBarRenderer renderer = new XYBarRenderer();
        renderer.setSeriesPaint(0, Color.lightGray);
        renderer.setSeriesPaint(1, new Color(100, 250, 100));
        renderer.setBarPainter(new ReverseGradientXYBarPainter());
        renderer.setDrawBarOutline(true);
        renderer.setShadowXOffset(2);
        renderer.setShadowYOffset(2);
        renderer.setUseYInterval(true);

        Collection<XYPlot> plots = new ArrayList<>();
        for (Fluent fluent : ps.getValues()) {
            XYIntervalSeriesCollection collection = new XYIntervalSeriesCollection();
            XYIntervalSeries true_series = new XYIntervalSeries("True");
            XYIntervalSeries false_series = new XYIntervalSeries("False");
            collection.addSeries(true_series);
            collection.addSeries(false_series);

            for (Literal lit : fluent.getValues()) {
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

            XYPlot plot = new XYPlot(collection, null, new NumberAxis(""), renderer);
            plot.getRangeAxis().setVisible(false);
            plot.setDatasetRenderingOrder(DatasetRenderingOrder.FORWARD);
            plot.addAnnotation(new XYTextAnnotation(fluent.getName(), 0, 1));

            plots.add(plot);
        }

        return plots;
    }
}