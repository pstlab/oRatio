const color_interpolator = d3.scaleSequential(d3.interpolateRdYlGn).domain([15, 0]);

export class Graph {

    constructor(svg, tooltip) {
        this.nodes = [];
        this.node_map = new Map();
        this.links = [];

        this.graph_g = svg.append('g');

        const graph_box = svg.node().getBoundingClientRect();
        const graph_width = graph_box.width, graph_height = graph_box.height;

        const graph_zoom = d3.zoom().on('zoom', event => graph_g.attr('transform', event.transform));
        svg.call(graph_zoom);

        svg.append('svg:defs').append('svg:marker')
            .attr('id', 'triangle')
            .attr('viewBox', '0 -5 10 10')
            .attr('refX', 8)
            .attr('refY', 0)
            .attr('markerWidth', 7)
            .attr('markerHeight', 7)
            .attr('orient', 'auto')
            .append('path')
            .attr('d', 'M0,-5L10,0L0,5');

        this.simulation = d3.forceSimulation(nodes)
            .force('link', d3.forceLink().id(d => d.id).distance(70))
            .force('charge', d3.forceManyBody().strength(-70))
            .force('center', d3.forceCenter(graph_width / 2, graph_height / 2));

        this.tooltip = tooltip;
    }

    reset(graph) {
        nodes.length = 0;
        links.length = 0;
        node_map.clear();

        graph.flaws.forEach(f => {
            f.type = 'flaw';
            nodes.push(f);
            node_map.set(f.id, f);
            f.causes.forEach(c => links.push({ source: f.id, target: c, state: f.state }));
        });
        graph.resolvers.forEach(r => {
            r.type = 'resolver';
            nodes.push(r);
            node_map.set(r.id, r);
            links.push({ source: r.id, target: r.effect, state: r.state });
            r.preconditions.filter(pre => !links.find(l => l.source === pre && l.target === r.id)).forEach(pre => links.push({ source: pre, target: r.id, state: node_map.get(pre).state }));
        });
        updateGraph();
        nodes.filter(n => n.type === 'resolver').forEach(r => update_resolver_cost(r));
        updateGraph();
    }

    flaw_created(flaw) {
        flaw.type = 'flaw';
        nodes.push(flaw);
        node_map.set(flaw.id, flaw);
        flaw.causes.forEach(c => links.push({ source: flaw.id, target: c, state: flaw.state }));
        updateGraph();
        flaw.causes.forEach(c => update_resolver_cost(node_map.get(c)));
        updateGraph();
    }

    flaw_state_changed(change) {
        node_map.get(change.id).state = change.state;
        links.filter(l => l.source.id === change.id).forEach(l => l.state = change.state);
        updateGraph();
    }

    flaw_cost_changed(change) {
        const f_node = node_map.get(change.id);
        f_node.cost = change.cost;
        links.filter(l => l.source.id == f_node.id).forEach(out_link => update_resolver_cost(out_link.target));
        updateGraph();
    }

    flaw_position_changed(change) {
        node_map.get(change.id).position = change.position;
        updateGraph();
    }

    current_flaw(current) {
        if (current_flaw) current_flaw.current = false;
        if (current_resolver) { current_resolver.current = false; current_resolver = undefined; }
        const f_node = node_map.get(current.id);
        f_node.current = true;
        current_flaw = f_node;
        updateGraph();
    }

    resolver_created(resolver) {
        resolver.type = 'resolver';
        nodes.push(resolver);
        node_map.set(resolver.id, resolver);
        links.push({ source: resolver.id, target: resolver.effect, state: resolver.state });
        updateGraph();
    }

    resolver_state_changed(change) {
        node_map.get(change.id).state = change.state;
        links.filter(l => l.source.id === change.id).forEach(l => l.state = change.state);
        updateGraph();
    }

    current_resolver(current) {
        if (current_resolver) current_resolver.current = false;
        const r_node = node_map.get(current.id);
        r_node.current = true;
        current_resolver = r_node;
        updateGraph();
    }

    causal_link_added(link) {
        links.push({ source: link.flaw_id, target: link.resolver_id, state: node_map.get(link.flaw_id).state });
        updateGraph();
        update_resolver_cost(node_map.get(link.resolver_id));
        updateGraph();
    }

    update() {
        const n_group = graph_g.selectAll('g').data(nodes, d => d.id).join(
            enter => {
                const g = enter.append('g').attr('cursor', 'grab');
                g.append('rect').attr('width', 30).attr('x', -15).attr('height', 10).attr('y', -5).attr('rx', d => radius(d)).attr('ry', d => radius(d)).style('fill', d => node_color(d)).style('stroke-dasharray', d => stroke_dasharray(d)).transition().duration(500).style('stroke', d => stroke(d)).style('stroke-width', d => stroke_width(d));
                g.append('text').attr('y', -7).text(d => d.type === 'flaw' ? flaw_label(d) : resolver_label(d)).style('text-anchor', 'middle');;
                g.on('mouseover', (event, d) => tooltip.html(d.type === 'flaw' ? flaw_tooltip(d) : resolver_tooltip(d)).transition().duration(200).style('opacity', .9))
                    .on('mousemove', event => tooltip.style('left', (event.pageX) + 'px').style('top', (event.pageY - 28) + 'px'))
                    .on('mouseout', event => tooltip.transition().duration(500).style('opacity', 0))
                    .on('click', (event, d) => { d.fx = null; d.fy = null; });
                g.call(d3.drag().on('start', dragstarted).on('drag', dragged).on('end', dragended));
                return g;
            },
            update => {
                update.select('rect').style('fill', d => node_color(d)).style('stroke-dasharray', d => stroke_dasharray(d)).transition().duration(500).style('stroke', d => stroke(d)).style('stroke-width', d => stroke_width(d));
                return update;
            }
        );
        const l_group = graph_g.selectAll('line').data(links).join(
            enter => {
                return enter.append('line').attr('stroke', 'black').style('stroke-dasharray', d => stroke_dasharray(d));
            },
            update => {
                update.style('stroke-dasharray', d => stroke_dasharray(d));
                return update;
            }
        );

        simulation.nodes(nodes).on('tick', () => {
            n_group.attr('transform', d => `translate(${d.x}, ${d.y})`);
            l_group.each(l => {
                let src = intersection({ x: l.source.x - 15, y: l.source.y - 5 }, { x: l.source.x - 15, y: l.source.y + 5 }, { x: l.source.x, y: l.source.y }, { x: l.target.x, y: l.target.y });
                if (!src) src = intersection({ x: l.source.x - 15, y: l.source.y + 5 }, { x: l.source.x + 15, y: l.source.y + 5 }, { x: l.source.x, y: l.source.y }, { x: l.target.x, y: l.target.y });
                if (!src) src = intersection({ x: l.source.x + 15, y: l.source.y + 5 }, { x: l.source.x + 15, y: l.source.y - 5 }, { x: l.source.x, y: l.source.y }, { x: l.target.x, y: l.target.y });
                if (!src) src = intersection({ x: l.source.x + 15, y: l.source.y - 5 }, { x: l.source.x - 15, y: l.source.y - 5 }, { x: l.source.x, y: l.source.y }, { x: l.target.x, y: l.target.y });

                if (src) {
                    l.x1 = src.x;
                    l.y1 = src.y;
                } else {
                    l.x1 = l.source.x;
                    l.y1 = l.source.y;
                }

                let trgt = intersection({ x: l.target.x - 17, y: l.target.y - 7 }, { x: l.target.x - 17, y: l.target.y + 7 }, { x: l.source.x, y: l.source.y }, { x: l.target.x, y: l.target.y });
                if (!trgt) trgt = intersection({ x: l.target.x - 17, y: l.target.y + 7 }, { x: l.target.x + 17, y: l.target.y + 7 }, { x: l.source.x, y: l.source.y }, { x: l.target.x, y: l.target.y });
                if (!trgt) trgt = intersection({ x: l.target.x + 17, y: l.target.y + 7 }, { x: l.target.x + 17, y: l.target.y - 7 }, { x: l.source.x, y: l.source.y }, { x: l.target.x, y: l.target.y });
                if (!trgt) trgt = intersection({ x: l.target.x + 17, y: l.target.y - 7 }, { x: l.target.x - 17, y: l.target.y - 7 }, { x: l.source.x, y: l.source.y }, { x: l.target.x, y: l.target.y });

                if (trgt) {
                    l.x2 = trgt.x;
                    l.y2 = trgt.y;
                } else {
                    l.x1 = l.target.x;
                    l.y1 = l.target.y;
                }
            }).attr('x1', d => d.x1).attr('y1', d => d.y1).attr('x2', d => d.x2).attr('y2', d => d.y2).attr('marker-end', 'url(#triangle)');
        });
        simulation.force('link').links(links);

        simulation.restart();
        simulation.alpha(0.3);
    }
}

function dragstarted(event, d) {
    if (!event.active) simulation.alphaTarget(0.3).restart();
    d3.select(this).attr('cursor', 'grabbing');
}

function dragged(event, d) {
    d.fx = event.x;
    d.fy = event.y;
    tooltip.style('left', (event.x) + 'px').style('top', (event.y - 28) + 'px');
}

function dragended(event, d) {
    if (!event.active) simulation.alphaTarget(0);
    d.fx = d.x;
    d.fy = d.y;
    d3.select(this).attr('cursor', 'grab');
}

function intersection(p0, p1, p2, p3) {
    const s1_x = p1.x - p0.x, s1_y = p1.y - p0.y, s2_x = p3.x - p2.x, s2_y = p3.y - p2.y;

    const s = (-s1_y * (p0.x - p2.x) + s1_x * (p0.y - p2.y)) / (-s2_x * s1_y + s1_x * s2_y);
    const t = (s2_x * (p0.y - p2.y) - s2_y * (p0.x - p2.x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        return { x: p0.x + (t * s1_x), y: p0.y + (t * s1_y) };
    else
        return undefined;
}

function radius(n) {
    switch (n.type) {
        case 'flaw':
            return 1;
        case 'resolver':
            return 4;
        default:
            break;
    }
}

function stroke(n) {
    return n.current ? '#ff00ff' : 'black';
}

function stroke_width(n) {
    return n.current ? '2' : '1';
}

function stroke_dasharray(n) {
    switch (n.state) {
        case 0: // False
            return '1';
        case 1: // True
            return '1000';
        case 2: // Undefined
            return '2';
        default:
            break;
    }
}

function update_resolver_cost(resolver) {
    let c_cost = Number.NEGATIVE_INFINITY;
    links.filter(l => l.target.id == resolver.id).forEach(in_link => {
        if (c_cost < in_link.source.cost)
            c_cost = in_link.source.cost;
    });
    resolver.cost = c_cost == Number.NEGATIVE_INFINITY ? resolver.cost : resolver.cost + c_cost;
}

function node_color(n) {
    switch (n.state) {
        case 0: // False
            return '#d9d9d9';
        default:
            return color_interpolator(n.cost);
    }
}

function flaw_label(flaw) {
    switch (flaw.label.type) {
        case 'fact':
            return 'fact \u03C3' + flaw.label.sigma + ' ' + flaw.label.predicate;
        case 'goal':
            return 'goal \u03C3' + flaw.label.sigma + ' ' + flaw.label.predicate;
        case 'enum':
            return 'enum';
        case 'bool':
            return 'bool';
        default:
            return '\u03C6' + flaw.label.phi + ' ' + flaw.label.type;
    }
}

function flaw_tooltip(flaw) { return '\u03C6' + flaw.label.phi + ', cost: ' + flaw.cost + ', pos: ' + flaw.position.min; }

function resolver_label(resolver) {
    if (resolver.label.type)
        switch (resolver.label.type) {
            case 'activate':
                return 'activate';
            case 'unify':
                return 'unify';
            default:
                return '\u03C1' + resolver.label.rho + ' ' + resolver.label.type;
        }
    return '\u03C1' + resolver.label.rho;
}

function resolver_tooltip(resolver) { return '\u03C1' + resolver.label.rho + ', cost: ' + resolver.cost; }