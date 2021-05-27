const color_interpolator = d3.scaleSequential(d3.interpolateRdYlGn).domain([15, 0]);

export class GraphData {

    constructor() {
        this.nodes = [];
        this.node_map = new Map();
        this.links = [];
    }

    reset(graph) {
        this.nodes.length = 0;
        this.links.length = 0;
        this.node_map.clear();

        graph.flaws.forEach(f => {
            f.type = 'flaw';
            f.label = JSON.parse(f.label);
            if (f.cost)
                f.cost = (f.cost.num / f.cost.den);
            else
                f.cost = Number.POSITIVE_INFINITY;
            f.graph = this;
            this.nodes.push(f);
            this.node_map.set(f.id, f);
            f.causes.forEach(c => this.links.push({ source: f.id, target: c, state: f.state }));
        });
        graph.resolvers.forEach(r => {
            r.type = 'resolver';
            r.label = JSON.parse(r.label);
            r.intrinsic_cost = r.cost.num / r.cost.den;
            r.cost = r.intrinsic_cost;
            r.graph = this;
            this.nodes.push(r);
            this.node_map.set(r.id, r);
            this.links.push({ source: r.id, target: r.effect, state: r.state });
            r.preconditions.filter(pre => !this.links.find(l => l.source === pre && l.target === r.id)).forEach(pre => this.links.push({ source: pre, target: r.id, state: this.node_map.get(pre).state }));
        });

        this.nodes.filter(n => n.type === 'resolver').forEach(r => this.update_resolver_cost(r));
    }

    solution_found() {
        if (this.c_flaw) this.c_flaw.current = false;
        if (this.c_resolver) { this.c_resolver.current = false; this.c_resolver = undefined; }
    }

    flaw_created(flaw) {
        flaw.type = 'flaw';
        flaw.label = JSON.parse(flaw.label);
        if (flaw.cost)
            flaw.cost = (flaw.cost.num / flaw.cost.den);
        else
            flaw.cost = Number.POSITIVE_INFINITY;
        flaw.graph = this;
        this.nodes.push(flaw);
        this.node_map.set(flaw.id, flaw);
        flaw.causes.forEach(c => this.links.push({ source: flaw.id, target: c, state: flaw.state }));
        flaw.causes.forEach(c => this.update_resolver_cost(this.node_map.get(c)));
    }

    flaw_state_changed(change) {
        this.node_map.get(change.id).state = change.state;
        this.links.filter(l => l.source.id === change.id).forEach(l => l.state = change.state);
    }

    flaw_cost_changed(change) {
        change.cost = change.cost.num / change.cost.den;
        const f_node = this.node_map.get(change.id);
        f_node.cost = change.cost;
        this.links.filter(l => l.source.id == f_node.id).forEach(out_link => this.update_resolver_cost(out_link.target));
    }

    flaw_position_changed(change) {
        this.node_map.get(change.id).position = change.position;
    }

    current_flaw(current) {
        if (this.c_flaw) this.c_flaw.current = false;
        if (this.c_resolver) { this.c_resolver.current = false; this.c_resolver = undefined; }
        const f_node = this.node_map.get(current.id);
        f_node.current = true;
        this.c_flaw = f_node;
    }

    resolver_created(resolver) {
        resolver.type = 'resolver';
        resolver.label = JSON.parse(resolver.label);
        resolver.intrinsic_cost = resolver.cost.num / resolver.cost.den;
        resolver.cost = resolver.intrinsic_cost;
        resolver.graph = this;
        this.nodes.push(resolver);
        this.node_map.set(resolver.id, resolver);
        this.links.push({ source: resolver.id, target: resolver.effect, state: resolver.state });
    }

    resolver_state_changed(change) {
        this.node_map.get(change.id).state = change.state;
        this.links.filter(l => l.source.id === change.id || l.target.id === change.id).forEach(l => l.state = change.state);
    }

    current_resolver(current) {
        if (this.c_resolver) this.c_resolver.current = false;
        const r_node = this.node_map.get(current.id);
        r_node.current = true;
        this.c_resolver = r_node;
    }

    causal_link_added(link) {
        this.links.push({ source: link.flaw, target: link.resolver, state: this.node_map.get(link.flaw).state });
        this.update_resolver_cost(this.node_map.get(link.resolver));
    }

    update_resolver_cost(resolver) {
        let c_cost = Number.NEGATIVE_INFINITY;
        this.links.filter(l => l.target.id == resolver.id).forEach(in_link => {
            if (c_cost < in_link.source.cost)
                c_cost = in_link.source.cost;
        });
        resolver.cost = c_cost == Number.NEGATIVE_INFINITY ? resolver.intrinsic_cost : resolver.intrinsic_cost + c_cost;
    }
}

export class Graph {

    constructor(graph_id = 'graph', width = window.innerWidth, height = window.innerHeight) {
        const svg = d3.select('#' + graph_id).append('svg').attr('viewBox', '0 0 ' + width + ' ' + height);

        this.graph_g = svg.append('g');

        const graph_zoom = d3.zoom().on('zoom', event => this.graph_g.attr('transform', event.transform));
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
            .attr('d', 'M0,-5L10,0L0,5')
            .attr('stroke', 'dimgray')
            .attr('fill', 'dimgray');

        this.simulation = d3.forceSimulation()
            .force('link', d3.forceLink().id(d => d.id).distance(70))
            .force('charge', d3.forceManyBody().strength(-70))
            .force('center', d3.forceCenter(width / 2, height / 2));

        this.tooltip = d3.select('.tooltip');
    }

    update(data) {
        data.nodes.forEach(n => n.graph = this);

        const l_group = this.graph_g.selectAll('line').data(data.links).join(
            enter => {
                return enter.append('line').attr('stroke', 'dimgray').style('stroke-dasharray', d => stroke_dasharray(d));
            },
            update => {
                update.style('stroke-dasharray', d => stroke_dasharray(d));
                return update;
            }
        );
        const n_group = this.graph_g.selectAll('g').data(data.nodes, d => d.id).join(
            enter => {
                const g = enter.append('g').attr('cursor', 'grab');
                g.append('rect').attr('width', 30).attr('x', -15).attr('height', 10).attr('y', -5).attr('rx', d => radius(d)).attr('ry', d => radius(d)).style('fill', d => node_color(d)).style('fill-opacity', d => node_opacity(d)).style('stroke-dasharray', d => stroke_dasharray(d)).style('opacity', d => node_opacity(d)).transition().duration(500).style('stroke', d => stroke(d)).style('stroke-width', d => stroke_width(d));
                g.append('text').attr('y', -8).text(d => d.type === 'flaw' ? flaw_label(d) : resolver_label(d)).style('text-anchor', 'middle').style('opacity', d => node_opacity(d));
                g.on('mouseover', (event, d) => this.tooltip.html(d.type === 'flaw' ? flaw_tooltip(d) : resolver_tooltip(d)).transition().duration(200).style('opacity', .9))
                    .on('mousemove', event => this.tooltip.style('left', (event.pageX) + 'px').style('top', (event.pageY - 28) + 'px'))
                    .on('mouseout', event => this.tooltip.transition().duration(500).style('opacity', 0))
                    .on('click', (event, d) => { d.fx = null; d.fy = null; });
                g.call(d3.drag().on('start', dragstarted).on('drag', dragged).on('end', dragended));
                return g;
            },
            update => {
                update.select('rect').style('fill', d => node_color(d)).style('fill-opacity', d => node_opacity(d)).style('stroke-dasharray', d => stroke_dasharray(d)).style('opacity', d => node_opacity(d)).transition().duration(500).style('stroke', d => stroke(d)).style('stroke-width', d => stroke_width(d));
                update.select('text').style('opacity', d => node_opacity(d));
                return update;
            }
        );

        this.simulation.nodes(data.nodes).on('tick', () => {
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
        this.simulation.force('link').links(data.links);

        this.simulation.restart();
        this.simulation.alpha(0.3);
    }
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
    return n.current ? '#ff00ff' : '#262626';
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

function dragstarted(event, d) {
    if (!event.active) d.graph.simulation.alphaTarget(0.3).restart();
    d3.select(this).attr('cursor', 'grabbing');
}

function dragged(event, d) {
    d.fx = event.x;
    d.fy = event.y;
    d.graph.tooltip.style('left', (event.x) + 'px').style('top', (event.y - 28) + 'px');
}

function dragended(event, d) {
    if (!event.active) d.graph.simulation.alphaTarget(0);
    d.fx = d.x;
    d.fy = d.y;
    d3.select(this).attr('cursor', 'grab');
}

function node_color(n) {
    switch (n.state) {
        case 0: // False
            return '#d9d9d9';
        default:
            return color_interpolator(n.cost);
    }
}

function node_opacity(n) {
    switch (n.state) {
        case 0: // False
            return 0.5;
        default:
            return 1;
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
            switch (flaw.label.phi) {
                case 'b0':
                case '\u00ACb0':
                    return flaw.label.type;
                default:
                    return flaw.label.phi.replace('b', '\u03C6') + ' ' + flaw.label.type;
            }
    }
}

function flaw_tooltip(flaw) {
    switch (flaw.label.phi) {
        case 'b0':
        case '\u00ACb0':
            return 'cost: ' + flaw.cost + ', pos: ' + flaw.position.min;
        default:
            return flaw.label.phi.replace('b', '\u03C6') + ', cost: ' + flaw.cost + ', pos: ' + flaw.position.min;
    }
}

function resolver_label(resolver) {
    if (resolver.label.type)
        switch (resolver.label.type) {
            case 'activate':
                return 'activate';
            case 'unify':
                return 'unify';
            case 'assignment':
                return resolver.label.val;
            default:
                switch (resolver.label.rho) {
                    case 'b0':
                    case '\u00ACb0':
                        return resolver.label.type;
                    default:
                        return resolver.label.rho.replace('b', '\u03C1') + ' ' + resolver.label.type;
                }
        }
    switch (resolver.label.rho) {
        case 'b0':
            return '\u22A4';
        case '\u00ACb0':
            return '\u22A5';
        default:
            return resolver.label.rho.replace('b', '\u03C1');
    }
}

function resolver_tooltip(resolver) {
    switch (resolver.label.rho) {
        case 'b0':
        case '\u00ACb0':
            return 'cost: ' + resolver.cost;
        default:
            return resolver.label.rho.replace('b', '\u03C1') + ', cost: ' + resolver.cost;
    }
}