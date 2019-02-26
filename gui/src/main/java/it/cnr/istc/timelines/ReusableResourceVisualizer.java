package it.cnr.istc.timelines;

import java.awt.BasicStroke;
import java.awt.Color;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import org.jfree.chart.annotations.XYTextAnnotation;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.DatasetRenderingOrder;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYStepRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import it.cnr.istc.oratio.timelines.ReusableResource;
import it.cnr.istc.oratio.timelines.ReusableResource.RRValue;
import it.cnr.istc.oratio.timelines.Timeline;

/**
 * ReusableResourceVisualizer
 */
public class ReusableResourceVisualizer implements TimelineVisualizer {

    @Override
    public Collection<XYPlot> getPlots(Timeline<?> timeline) {
        assert timeline instanceof ReusableResource;
        ReusableResource rr = (ReusableResource) timeline;

        XYSeriesCollection collection = new XYSeriesCollection();
        XYSeries profile = new XYSeries("Profile");
        XYSeries capacity_series = new XYSeries("Capacity");

        profile.add(rr.getOrigin(), 0);
        List<RRValue> vals = rr.getValues();
        if (!vals.isEmpty())
            if (vals.get(0).getFrom().gt(rr.getOrigin()))
                profile.add(vals.get(0).getFrom(), vals.get(0).getUsage());
        for (RRValue val : rr.getValues()) {
            profile.add(val.getFrom(), val.getUsage());
            profile.add(val.getTo(), val.getUsage());
        }
        profile.add(rr.getHorizon(), 0);
        collection.addSeries(profile);

        capacity_series.add(rr.getOrigin(), rr.getCapacity());
        capacity_series.add(rr.getHorizon(), rr.getCapacity());
        collection.addSeries(capacity_series);

        XYStepRenderer renderer = new XYStepRenderer();
        renderer.setSeriesPaint(0, Color.BLUE);
        renderer.setSeriesToolTipGenerator(0, (XYDataset dataset, int series,
                int item) -> ((XYSeriesCollection) dataset).getSeries(series).getDataItem(item).getY().toString());
        renderer.setSeriesPaint(1, new Color(135, 0, 0));
        renderer.setSeriesStroke(1, new BasicStroke(1.5f));

        XYPlot plot = new XYPlot(collection, null, new NumberAxis(""), renderer);
        plot.setDatasetRenderingOrder(DatasetRenderingOrder.FORWARD);
        plot.addAnnotation(new XYTextAnnotation(rr.getName(), 0, 1));

        return Arrays.asList(plot);
    }
}