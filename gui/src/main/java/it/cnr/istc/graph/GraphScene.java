package it.cnr.istc.graph;

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

import it.cnr.istc.riddle.Core;
import javafx.embed.swing.SwingNode;
import javafx.scene.Scene;
import javafx.scene.layout.StackPane;
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
 * GraphScene
 */
public class GraphScene extends Scene implements GraphListener {

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
    private final Core core;
    private final Visualization vis = new Visualization();
    private final Graph g = new Graph(true);
    private final VisualGraph vg = vis.addGraph(GRAPH, g);
    private final Display display = new Display(vis);
    private final Map<String, Node> flaws = new HashMap<>();
    private final Map<String, Node> resolvers = new HashMap<>();
    private final Map<String, String> effects = new HashMap<>();
    private final Map<String, Collection<String>> flaw_resolvers = new HashMap<>();
    private String current_flaw;
    private String current_resolver;

    public GraphScene(final Core core) {
        super(new StackPane(new SwingNode()));
        this.core = core;

        Logger.getLogger("prefuse").setLevel(Level.OFF);

        g.getNodeTable().addColumn(VisualItem.LABEL, String.class);
        g.getNodeTable().addColumn(NODE_TYPE, String.class);
        g.getNodeTable().addColumn(NODE_COST, Double.class);
        g.getNodeTable().addColumn(NODE_STATE, Integer.class);
        g.getEdgeTable().addColumn(VisualItem.LABEL, String.class);
        g.getEdgeTable().addColumn(EDGE_STATE, Integer.class);

        vis.setInteractive(EDGES, null, false);
        vis.setValue(NODES, null, VisualItem.SHAPE, Constants.SHAPE_ELLIPSE);

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
        vis.setRendererFactory(drf);

        // adding decorators, one group for the nodes, one for the edges
        DECORATOR_SCHEMA.setDefault(VisualItem.TEXTCOLOR, ColorLib.gray(50));
        vis.addDecorators(EDGE_DECORATORS, EDGES, DECORATOR_SCHEMA);

        DECORATOR_SCHEMA.setDefault(VisualItem.TEXTCOLOR, ColorLib.gray(128));
        vis.addDecorators(NODE_DECORATORS, NODES, DECORATOR_SCHEMA);

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
        vis.putAction("layout", layout);

        // set up the display
        display.setHighQuality(true);
        display.addControlListener(new PanControl());
        display.addControlListener(new DragControl());
        display.addControlListener(new ZoomToFitControl());
        display.addControlListener(new WheelZoomControl());
        display.addControlListener(new ControlAdapter() {
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

        // TODO: create nodes and edges according to the current core..

        // set things running..
        vis.run("layout");

        ((SwingNode) getRoot().getChildrenUnmodifiable().get(0)).setContent(display);
    }

    /**
     * Set label positions. Labels are assumed to be DecoratorItem instances,
     * decorating their respective nodes. The layout simply gets the bounds of the
     * decorated node and assigns the label coordinates to the center of those
     * bounds.
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

    @Override
    public void flaw_created(FlawCreated flaw) {
        synchronized (vis) {
            assert !flaws.containsKey(flaw.flaw) : "the flaw already exists..";
            assert Arrays.stream(flaw.causes)
                    .allMatch(c -> resolvers.containsKey(c)) : "the flaw's cause does not exist: "
                            + Arrays.toString(flaw.causes) + resolvers;
            Node flaw_node = g.addNode();
            flaw_node.set(VisualItem.LABEL, flaw.label);
            flaw_node.set(NODE_TYPE, "flaw");
            flaw_node.set(NODE_COST, Double.NEGATIVE_INFINITY);
            flaw_node.set(NODE_STATE, flaw.state);
            flaws.put(flaw.flaw, flaw_node);
            for (String c : flaw.causes) {
                Edge c_edge = g.addEdge(flaw_node, resolvers.get(c));
                c_edge.set(EDGE_STATE, resolvers.get(c).get(NODE_STATE));
            }
        }
    }

    @Override
    public void flaw_state_changed(FlawStateChanged flaw) {
        synchronized (vis) {
            assert flaws.containsKey(flaw.flaw) : "the flaw does not exist..";
            Node flaw_node = flaws.get(flaw.flaw);
            flaw_node.set(NODE_STATE, flaw.state);
        }
    }

    @Override
    public void current_flaw(CurrentFlaw flaw) {
        synchronized (vis) {
            assert flaws.containsKey(flaw.flaw) : "the flaw does not exist..";
            if (current_flaw != null)
                vis.getVisualItem(NODES, flaws.get(current_flaw)).setHighlighted(false);
            if (current_resolver != null)
                vis.getVisualItem(NODES, resolvers.get(current_resolver)).setHighlighted(false);

            current_flaw = flaw.flaw;
            vis.getVisualItem(NODES, flaws.get(flaw.flaw)).setHighlighted(true);
        }
    }

    @Override
    public void resolver_created(ResolverCreated resolver) {
        synchronized (vis) {
            assert !resolvers.containsKey(resolver.resolver) : "the resolver already exists..";
            assert flaws.containsKey(resolver.effect) : "the resolver's solved flaw does not exist..";
            effects.put(resolver.resolver, resolver.effect);
            if (!flaw_resolvers.containsKey(resolver.effect))
                flaw_resolvers.put(resolver.effect, new ArrayList<>());
            flaw_resolvers.get(resolver.effect).add(resolver.resolver);
            Node resolver_node = g.addNode();
            resolver_node.set(VisualItem.LABEL, resolver.label);
            resolver_node.set(NODE_TYPE, "resolver");
            resolver_node.set(NODE_COST, -(double) resolver.cost.getNumerator() / resolver.cost.getDenominator());
            resolver_node.set(NODE_STATE, resolver.state);
            resolvers.put(resolver.resolver, resolver_node);
            Edge c_edge = g.addEdge(resolver_node, flaws.get(resolver.effect));
            c_edge.set(EDGE_STATE, resolver.state);
            flaws.get(resolver.effect).set(NODE_COST, flaw_resolvers.get(resolver.effect).stream()
                    .mapToDouble(res -> (Double) resolvers.get(res).get(NODE_COST)).max().getAsDouble());
        }
    }

    @Override
    @SuppressWarnings("unchecked")
    public void resolver_state_changed(ResolverStateChanged resolver) {
        synchronized (vis) {
            assert resolvers.containsKey(resolver.resolver) : "the resolver does not exist..";
            Node resolver_node = resolvers.get(resolver.resolver);
            resolver_node.set(NODE_STATE, resolver.state);
            Iterator<Edge> c_edges = resolver_node.edges();
            while (c_edges.hasNext())
                c_edges.next().set(EDGE_STATE, resolver.state);
        }
    }

    @Override
    public void resolver_cost_changed(ResolverCostChanged resolver) {
        synchronized (vis) {
            assert resolvers.containsKey(resolver.resolver) : "the resolver does not exist..";
            assert flaws.containsKey(effects.get(resolver.resolver)) : "the resolver's effect does not exist..";
            Node resolver_node = resolvers.get(resolver.resolver);
            resolver_node.set(NODE_COST, -(double) resolver.cost.getNumerator() / resolver.cost.getDenominator());
            flaws.get(effects.get(resolver.resolver)).set(NODE_COST, flaw_resolvers.get(effects.get(resolver.resolver))
                    .stream().mapToDouble(res -> (Double) resolvers.get(res).get(NODE_COST)).max().getAsDouble());
        }
    }

    @Override
    public void current_resolver(CurrentResolver resolver) {
        assert resolvers.containsKey(resolver.resolver) : "the resolver does not exist..";
        synchronized (vis) {
            current_resolver = resolver.resolver;
            vis.getVisualItem(NODES, resolvers.get(resolver.resolver)).setHighlighted(true);
        }
    }

    @Override
    public void causal_link_added(CausalLinkAdded causal_link) {
        synchronized (vis) {
            assert flaws.containsKey(causal_link.flaw) : "the flaw does not exist..";
            assert resolvers.containsKey(causal_link.resolver) : "the resolver does not exist..";
            Edge c_edge = g.addEdge(flaws.get(causal_link.flaw), resolvers.get(causal_link.resolver));
            c_edge.set(EDGE_STATE, resolvers.get(causal_link.resolver).get(NODE_STATE));
            flaws.get(effects.get(causal_link.resolver)).set(NODE_COST,
                    flaw_resolvers.get(effects.get(causal_link.resolver)).stream()
                            .mapToDouble(res -> (Double) resolvers.get(res).get(NODE_COST)).max().getAsDouble());
        }
    }
}