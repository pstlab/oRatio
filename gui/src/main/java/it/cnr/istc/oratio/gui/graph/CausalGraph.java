/*
 * Copyright (C) 2018 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
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
package it.cnr.istc.oratio.gui.graph;

import it.cnr.istc.oratio.Rational;
import java.awt.event.MouseEvent;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import prefuse.Constants;
import prefuse.Display;
import prefuse.Visualization;
import prefuse.action.ActionList;
import prefuse.action.RepaintAction;
import prefuse.action.assignment.ColorAction;
import prefuse.action.assignment.DataColorAction;
import prefuse.action.assignment.StrokeAction;
import prefuse.action.layout.Layout;
import prefuse.action.layout.graph.ForceDirectedLayout;
import prefuse.activity.Activity;
import prefuse.controls.ControlAdapter;
import prefuse.controls.DragControl;
import prefuse.controls.PanControl;
import prefuse.controls.WheelZoomControl;
import prefuse.controls.ZoomToFitControl;
import prefuse.data.Edge;
import prefuse.data.Graph;
import prefuse.data.Node;
import prefuse.data.Schema;
import prefuse.render.DefaultRendererFactory;
import prefuse.render.EdgeRenderer;
import prefuse.render.LabelRenderer;
import prefuse.util.ColorLib;
import prefuse.util.FontLib;
import prefuse.util.PrefuseLib;
import prefuse.util.StrokeLib;
import prefuse.visual.DecoratorItem;
import prefuse.visual.VisualGraph;
import prefuse.visual.VisualItem;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class CausalGraph extends Display {

    private static final String GRAPH = "graph";
    private static final String NODES = "graph.nodes";
    private static final String EDGES = "graph.edges";
    private static final String NODE_TYPE = "node_type";
    private static final String NODE_COST = "node_cost";
    private static final String NODE_STATE = "node_state";
    private static final String EDGE_STATE = "edge_state";
    private static final String EDGE_DECORATORS = "edgeDeco";
    private static final String NODE_DECORATORS = "nodeDeco";
    private static final Schema DECORATOR_SCHEMA = PrefuseLib.getVisualItemSchema();

    static {
        DECORATOR_SCHEMA.setDefault(VisualItem.INTERACTIVE, false);
        DECORATOR_SCHEMA.setDefault(VisualItem.TEXTCOLOR, ColorLib.gray(128));
        DECORATOR_SCHEMA.setDefault(VisualItem.FONT, FontLib.getFont("Tahoma", 7));
    }
    private final Graph g = new Graph(true);
    private final VisualGraph vg;
    private final Map<String, Node> flaws = new HashMap<>();
    private final Map<String, Node> resolvers = new HashMap<>();
    private final Map<String, String> effects = new HashMap<>();
    private final Map<String, Collection<String>> flaw_resolvers = new HashMap<>();
    private String current_flaw;
    private String current_resolver;

    public CausalGraph() {
        // initialize display and data
        super(new Visualization());

        Logger.getLogger("prefuse").setLevel(Level.OFF);

        g.getNodeTable().addColumn(VisualItem.LABEL, String.class);
        g.getNodeTable().addColumn(NODE_TYPE, String.class);
        g.getNodeTable().addColumn(NODE_COST, Double.class);
        g.getNodeTable().addColumn(NODE_STATE, Integer.class);
        g.getEdgeTable().addColumn(VisualItem.LABEL, String.class);
        g.getEdgeTable().addColumn(EDGE_STATE, Integer.class);

        // add visual data groups
        vg = m_vis.addGraph(GRAPH, g);

        m_vis.setInteractive(EDGES, null, false);
        m_vis.setValue(NODES, null, VisualItem.SHAPE, Constants.SHAPE_ELLIPSE);

        // set up the renderers
        // draw the nodes as basic shapes
        LabelRenderer flaw_renderer = new LabelRenderer(VisualItem.LABEL);
        flaw_renderer.setRoundedCorner(3, 3);
        LabelRenderer resolver_renderer = new LabelRenderer(VisualItem.LABEL);
        resolver_renderer.setRoundedCorner(15, 15);

        DefaultRendererFactory drf = new DefaultRendererFactory();
        drf.setDefaultRenderer(flaw_renderer);
        drf.add("ingroup('" + NODES + "')&&" + NODE_TYPE + "==\"resolver\"", resolver_renderer);
        drf.setDefaultEdgeRenderer(new EdgeRenderer(Constants.EDGE_TYPE_CURVE));
        m_vis.setRendererFactory(drf);

        // adding decorators, one group for the nodes, one for the edges
        DECORATOR_SCHEMA.setDefault(VisualItem.TEXTCOLOR, ColorLib.gray(50));
        m_vis.addDecorators(EDGE_DECORATORS, EDGES, DECORATOR_SCHEMA);

        DECORATOR_SCHEMA.setDefault(VisualItem.TEXTCOLOR, ColorLib.gray(128));
        m_vis.addDecorators(NODE_DECORATORS, NODES, DECORATOR_SCHEMA);

        // set up the visual operators
        // first set up all the color actions
        ColorAction nFill = new DataColorAction(NODES, NODE_COST, Constants.ORDINAL, VisualItem.FILLCOLOR,
                ColorLib.getHotPalette());
        nFill.add(VisualItem.HOVER, ColorLib.gray(200));
        nFill.add(VisualItem.HIGHLIGHT, ColorLib.rgb(255, 230, 230));
        nFill.add(NODE_STATE + " == " + 0, ColorLib.gray(235));

        ColorAction nStrokeColor = new ColorAction(NODES, VisualItem.STROKECOLOR);
        nStrokeColor.setDefaultColor(ColorLib.gray(100));
        nStrokeColor.add(VisualItem.HOVER, ColorLib.gray(200));

        StrokeAction nStroke = new StrokeAction(NODES, StrokeLib.getStroke(5));
        nStroke.add(NODE_STATE + " == " + 0, StrokeLib.getStroke(0.1f, StrokeLib.DOTS));
        nStroke.add(NODE_STATE + " == " + 1, StrokeLib.getStroke(0.5f));
        nStroke.add(NODE_STATE + " == " + 2, StrokeLib.getStroke(1, StrokeLib.DASHES));

        ColorAction eStrokeColor = new ColorAction(EDGES, VisualItem.STROKECOLOR);
        eStrokeColor.setDefaultColor(ColorLib.gray(100));

        StrokeAction eStroke = new StrokeAction(EDGES, StrokeLib.getStroke(5));
        eStroke.add(EDGE_STATE + " == " + 0, StrokeLib.getStroke(0.1f, StrokeLib.DOTS));
        eStroke.add(EDGE_STATE + " == " + 1, StrokeLib.getStroke(0.5f));
        eStroke.add(EDGE_STATE + " == " + 2, StrokeLib.getStroke(1, StrokeLib.DASHES));

        ColorAction eFill = new ColorAction(EDGES, VisualItem.FILLCOLOR);
        eFill.setDefaultColor(ColorLib.gray(100));

        // bundle the color actions
        ActionList colors = new ActionList();
        colors.add(nFill);
        colors.add(nStrokeColor);
        colors.add(nStroke);
        colors.add(eStrokeColor);
        colors.add(eStroke);
        colors.add(eFill);

        // now create the main layout routine
        ActionList layout = new ActionList(Activity.INFINITY);
        layout.add(colors);
        layout.add(new LabelLayout2(EDGE_DECORATORS));
        layout.add(new LabelLayout2(NODE_DECORATORS));
        layout.add(new ForceDirectedLayout(GRAPH, false));
        layout.add(new RepaintAction());
        m_vis.putAction("layout", layout);

        // set up the display
        setHighQuality(true);
        addControlListener(new PanControl());
        addControlListener(new DragControl());
        addControlListener(new ZoomToFitControl());
        addControlListener(new WheelZoomControl());
        addControlListener(new ControlAdapter() {
            @Override
            public void itemEntered(VisualItem vi, MouseEvent me) {
                Display d = (Display) me.getSource();
                if (vi.getSourceTuple() instanceof Node) {
                    Node nodeData = (Node) vi.getSourceTuple();
                    String t_text = "";
                    switch ((String) nodeData.get(NODE_TYPE)) {
                        case "flaw":
                            t_text += Character.toString('\u03C6');
                            break;
                        case "resolver":
                            t_text += Character.toString('\u03C1');
                            break;
                    }
                    t_text += ": ";
                    switch ((int) nodeData.get(NODE_STATE)) {
                        case 0:
                            t_text += "False";
                            break;
                        case 1:
                            t_text += "True";
                            break;
                        case 2:
                            t_text += "Undefined";
                            break;
                        default:
                            break;
                    }
                    t_text += ", cost: " + (-(Double) nodeData.get(NODE_COST));
                    d.setToolTipText(t_text);
                }
            }

            @Override
            public void itemExited(VisualItem vi, MouseEvent me) {
                Display d = (Display) me.getSource();
                d.setToolTipText(null);
            }

            @Override
            public void itemClicked(VisualItem vi, MouseEvent me) {
                if (vi.getSourceTuple() instanceof Node) {
                    Node nodeData = (Node) vi.getSourceTuple();
                    switch ((String) nodeData.get(NODE_TYPE)) {
                        case "flaw":
                            break;
                        case "resolver":
                            break;
                    }
                }
            }
        });

        // set things running
        m_vis.run("layout");
    }

    public void flaw_created(final String flaw, final String[] causes, final String label, final int state) {
        synchronized (m_vis) {
            assert !flaws.containsKey(flaw) : "the flaw already exists..";
            assert Arrays.stream(causes)
                    .allMatch(c -> resolvers.containsKey(c)) : "the flaw's cause does not exist: "
                    + Arrays.toString(causes) + resolvers;
            Node flaw_node = g.addNode();
            flaw_node.set(VisualItem.LABEL, label);
            flaw_node.set(NODE_TYPE, "flaw");
            flaw_node.set(NODE_COST, Double.NEGATIVE_INFINITY);
            flaw_node.set(NODE_STATE, state);
            flaws.put(flaw, flaw_node);
            for (String c : causes) {
                Edge c_edge = g.addEdge(flaw_node, resolvers.get(c));
                c_edge.set(EDGE_STATE, resolvers.get(c).get(NODE_STATE));
            }
        }
    }

    public void flaw_state_changed(final String flaw, final int state) {
        synchronized (m_vis) {
            assert flaws.containsKey(flaw) : "the flaw does not exist..";
            Node flaw_node = flaws.get(flaw);
            flaw_node.set(NODE_STATE, state);
        }
    }

    public void current_flaw(final String flaw) {
        assert flaws.containsKey(flaw) : "the flaw does not exist..";
        synchronized (m_vis) {
            if (current_flaw != null) {
                m_vis.getVisualItem(NODES, flaws.get(current_flaw)).setHighlighted(false);
            }
            if (current_resolver != null) {
                m_vis.getVisualItem(NODES, resolvers.get(current_resolver)).setHighlighted(false);
            }

            current_flaw = flaw;
            m_vis.getVisualItem(NODES, flaws.get(flaw)).setHighlighted(true);
        }
    }

    public void resolver_created(final String resolver, final String effect, final Rational cost, final String label, final int state) {
        synchronized (m_vis) {
            assert !resolvers.containsKey(resolver) : "the resolver already exists..";
            assert flaws.containsKey(effect) : "the resolver's solved flaw does not exist..";
            effects.put(resolver, effect);
            if (!flaw_resolvers.containsKey(effect)) {
                flaw_resolvers.put(effect, new ArrayList<>());
            }
            flaw_resolvers.get(effect).add(resolver);
            Node resolver_node = g.addNode();
            resolver_node.set(VisualItem.LABEL, label);
            resolver_node.set(NODE_TYPE, "resolver");
            resolver_node.set(NODE_COST, -(double) cost.getNumerator() / cost.getDenominator());
            resolver_node.set(NODE_STATE, state);
            resolvers.put(resolver, resolver_node);
            Edge c_edge = g.addEdge(resolver_node, flaws.get(effect));
            c_edge.set(EDGE_STATE, state);
            flaws.get(effect).set(NODE_COST, flaw_resolvers.get(effect).stream()
                    .mapToDouble(res -> (Double) resolvers.get(res).get(NODE_COST)).max().getAsDouble());
        }
    }

    @SuppressWarnings("unchecked")
    public void resolver_state_changed(final String resolver, final int state) {
        synchronized (m_vis) {
            assert resolvers.containsKey(resolver) : "the resolver does not exist..";
            Node resolver_node = resolvers.get(resolver);
            resolver_node.set(NODE_STATE, state);
            Iterator<Edge> c_edges = resolver_node.edges();
            while (c_edges.hasNext()) {
                c_edges.next().set(EDGE_STATE, state);
            }
        }
    }

    public void resolver_cost_changed(final String resolver, final Rational cost) {
        synchronized (m_vis) {
            assert resolvers.containsKey(resolver) : "the resolver does not exist..";
            assert flaws.containsKey(effects.get(resolver)) : "the resolver's effect does not exist..";
            Node resolver_node = resolvers.get(resolver);
            resolver_node.set(NODE_COST, -(double) cost.getNumerator() / cost.getDenominator());
            flaws.get(effects.get(resolver)).set(NODE_COST, flaw_resolvers.get(effects.get(resolver)).stream()
                    .mapToDouble(res -> (Double) resolvers.get(res).get(NODE_COST)).max().getAsDouble());
        }
    }

    public void current_resolver(final String resolver) {
        assert resolvers.containsKey(resolver) : "the resolver does not exist..";
        synchronized (m_vis) {
            current_resolver = resolver;
            m_vis.getVisualItem(NODES, resolvers.get(resolver)).setHighlighted(true);
        }
    }

    public void causal_link_added(final String flaw, final String resolver) {
        synchronized (m_vis) {
            assert flaws.containsKey(flaw) : "the flaw does not exist..";
            assert resolvers.containsKey(resolver) : "the resolver does not exist..";
            Edge c_edge = g.addEdge(flaws.get(flaw), resolvers.get(resolver));
            c_edge.set(EDGE_STATE, resolvers.get(resolver).get(NODE_STATE));
            flaws.get(effects.get(resolver)).set(NODE_COST, flaw_resolvers.get(effects.get(resolver)).stream()
                    .mapToDouble(res -> (Double) resolvers.get(res).get(NODE_COST)).max().getAsDouble());
        }
    }

    /**
     * Set label positions. Labels are assumed to be DecoratorItem instances,
     * decorating their respective nodes. The layout simply gets the bounds of
     * the decorated node and assigns the label coordinates to the center of
     * those bounds.
     */
    private static class LabelLayout2 extends Layout {

        LabelLayout2(String group) {
            super(group);
        }

        @Override
        public void run(double frac) {
            Iterator<?> iter = m_vis.items(m_group);
            while (iter.hasNext()) {
                DecoratorItem decorator = (DecoratorItem) iter.next();
                VisualItem decoratedItem = decorator.getDecoratedItem();
                Rectangle2D bounds = decoratedItem.getBounds();

                double x = bounds.getCenterX();
                double y = bounds.getCenterY();

                setX(decorator, null, x);
                setY(decorator, null, y);
            }
        }
    }
}
