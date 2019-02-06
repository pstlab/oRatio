package it.cnr.istc.timelines;

import java.awt.Color;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.DatasetRenderingOrder;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYBarRenderer;
import org.jfree.data.xy.XYIntervalSeries;
import org.jfree.data.xy.XYIntervalSeriesCollection;

import it.cnr.istc.riddle.Atom;
import it.cnr.istc.riddle.Core;
import it.cnr.istc.riddle.Field;
import it.cnr.istc.riddle.Item;
import it.cnr.istc.riddle.Predicate;
import it.cnr.istc.riddle.Item.EnumItem;
import it.cnr.istc.riddle.Item.BoolItem.LBool;
import it.cnr.istc.utils.CartesianProductGenerator;

/**
 * PropositionalStateVisualizer
 */
public class PropositionalStateVisualizer implements TimelineVisualizer {

    private final Core core;

    PropositionalStateVisualizer(Core core) {
        this.core = core;
    }

    @Override
    public Collection<XYPlot> getPlots(Item itm, Collection<Atom> atoms) {
        Map<String, XYPlot> plots = new LinkedHashMap<>();

        XYBarRenderer renderer = new XYBarRenderer();
        renderer.setSeriesPaint(0, Color.lightGray);
        renderer.setSeriesPaint(1, new Color(100, 250, 100));
        renderer.setBarPainter(new ReverseGradientXYBarPainter());
        renderer.setDrawBarOutline(true);
        renderer.setShadowXOffset(2);
        renderer.setShadowYOffset(2);
        renderer.setUseYInterval(true);

        for (Predicate p : itm.getType().getPredicates().values()) {
            Item[][] itms = new Item[p.getFields().size() - 2][];
            int i = 0;
            for (Field fld : p.getFields().values())
                if (!fld.getName().equals("tau") && !fld.getName().equals("polarity"))
                    itms[i++] = fld.getType().getInstances().toArray(Item[]::new);

            for (Item[] c_itms : new CartesianProductGenerator<>(itms)) {
                XYIntervalSeriesCollection collection = new XYIntervalSeriesCollection();
                collection.addSeries(new XYIntervalSeries("True"));
                collection.addSeries(new XYIntervalSeries("False"));

                XYPlot plot = new XYPlot(collection, null, new NumberAxis(""), renderer);
                plot.getRangeAxis().setVisible(false);
                plot.setDatasetRenderingOrder(DatasetRenderingOrder.FORWARD);
                plots.put(p.getName() + "("
                        + Stream.of(c_itms).map(c_itm -> core.guessName(c_itm)).collect(Collectors.joining(", ")) + ")",
                        plot);
            }
        }

        for (Atom atom : atoms) {
            double start_pulse = ((Item.ArithItem) atom.getExpr("start")).getValue().doubleValue();
            double end_pulse = ((Item.ArithItem) atom.getExpr("end")).getValue().doubleValue();

            ((XYIntervalSeriesCollection) plots.get(atom.getType().getName() + "("
                    + atom.getType().getFields().values().stream()
                            .filter(fld -> !fld.getName().equals("tau") && !fld.getName().equals("polarity"))
                            .map(fld -> atom.getExpr(fld.getName())).map(c_itm -> {
                                if (c_itm instanceof EnumItem) {
                                    if (((EnumItem) c_itm).getVals().length == 1)
                                        return core.guessName(((EnumItem) c_itm).getVals()[0]);
                                    else
                                        return Stream.of(((EnumItem) c_itm).getVals()).map(i -> core.guessName(i))
                                                .collect(Collectors.joining(", "));
                                } else
                                    return core.guessName(c_itm);
                            }).collect(Collectors.joining(", "))
                    + ")").getDataset())
                            .getSeries((((Item.BoolItem) atom.getExpr("polarity")).getValue() == LBool.True) ? 0 : 1)
                            .add(start_pulse, start_pulse, end_pulse, 0, 0, 1);
        }

        return plots.values();
    }
}