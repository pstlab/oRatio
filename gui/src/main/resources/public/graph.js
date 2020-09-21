const nodes = [];
const node_map = {}
const links = [];

let current_flaw, current_resolver;

const svg = d3.select('#graph').append('svg');
const g = svg.append('g');

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

const b_box = svg.node().getBoundingClientRect();
const width = b_box.width, height = b_box.height;

const c_zoom = d3.zoom().on('zoom', event => g.attr('transform', event.transform));
svg.call(c_zoom);

var color_interpolator = d3.scaleSequential().domain([15, 0]).interpolator(d3.interpolateRdYlGn);

var tooltip = d3.select('body').append('div').attr('class', 'tooltip').style('opacity', 0);

const simulation = d3.forceSimulation(nodes)
    .force('link', d3.forceLink().id(d => d.id).distance(60))
    .force('charge', d3.forceManyBody().strength(-60))
    .force('center', d3.forceCenter(width / 2, height / 2));

updateGraph();

const ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/graph');
ws.onmessage = msg => {
    if (msg.data.startsWith('graph ')) {
        nodes.length = 0;
        links.length = 0;

        const c_graph = JSON.parse(msg.data.substring('graph '.length));
        c_graph.flaws.forEach(f => {
            const c_f = JSON.parse(f);
            c_f.label = JSON.parse(c_f.label);
            if (c_f.cost)
                c_f.cost = (c_f.cost.num / c_f.cost.den);
            else
                c_f.cost = Number.POSITIVE_INFINITY;
            c_f.type = 'flaw';

            nodes.push(c_f);
            node_map[c_f.id] = c_f;
            c_f.causes.forEach(c => links.push({ source: c_f.id, target: c, state: c_f.state }));
        });
        c_graph.resolvers.forEach(r => {
            const c_r = JSON.parse(r);
            c_r.label = JSON.parse(c_r.label);
            c_r.cost = c_r.intrinsic_cost.num / c_r.intrinsic_cost.den;
            c_r.type = 'resolver';

            nodes.push(c_r);
            node_map[c_r.id] = c_r;
            links.push({ source: c_r.id, target: c_r.effect, state: c_r.state });
            c_r.preconditions.filter(pre => !links.find(l => l.source === pre && l.target === c_r.id)).forEach(pre => links.push({ source: pre, target: c_r.id, state: node_map[pre].state }));
        });
        updateGraph();
        nodes.filter(n => n.type === 'resolver').forEach(r => update_resolver_cost(r));
        updateGraph();
    } else if (msg.data.startsWith('flaw_created ')) {
        const c_f = JSON.parse(msg.data.substring('flaw_created '.length));
        c_f.label = JSON.parse(c_f.label);
        if (c_f.cost)
            c_f.cost = (c_f.cost.num / c_f.cost.den);
        else
            c_f.cost = Number.POSITIVE_INFINITY;
        c_f.type = 'flaw';

        nodes.push(c_f);
        node_map[c_f.id] = c_f;
        c_f.causes.forEach(c => links.push({ source: c_f.id, target: c, state: c_f.state }));
        updateGraph();
        c_f.causes.forEach(c => update_resolver_cost(node_map[c]));
        updateGraph();
    } else if (msg.data.startsWith('flaw_state_changed ')) {
        const c_f = JSON.parse(msg.data.substring('flaw_state_changed '.length));
        node_map[c_f.id].state = c_f.state;
        links.filter(l => l.source.id === c_f.id).forEach(l => l.state = c_f.state);
        updateGraph();
    } else if (msg.data.startsWith('flaw_cost_changed ')) {
        const c_f = JSON.parse(msg.data.substring('flaw_cost_changed '.length));
        const f_node = node_map[c_f.id];
        f_node.cost = c_f.cost.num / c_f.cost.den;
        links.filter(l => l.source.id == f_node.id).forEach(out_link => update_resolver_cost(out_link.target));
        updateGraph();
    } else if (msg.data.startsWith('flaw_position_changed ')) {
        const c_f = JSON.parse(msg.data.substring('flaw_position_changed '.length));
        node_map[c_f.id].position = c_f.position;
        updateGraph();
    } else if (msg.data.startsWith('current_flaw ')) {
        const c_f = JSON.parse(msg.data.substring('current_flaw '.length));
        if (current_flaw) current_flaw.current = false;
        if (current_resolver) { current_resolver.current = false; current_resolver = undefined; }
        const f_node = node_map[c_f.id];
        f_node.current = true;
        current_flaw = f_node;
        updateGraph();
    } else if (msg.data.startsWith('resolver_created ')) {
        const c_r = JSON.parse(msg.data.substring('resolver_created '.length));
        c_r.label = JSON.parse(c_r.label);
        c_r.cost = c_r.intrinsic_cost.num / c_r.intrinsic_cost.den;
        c_r.type = 'resolver';

        nodes.push(c_r);
        node_map[c_r.id] = c_r;
        links.push({ source: c_r.id, target: c_r.effect, state: c_r.state });
        updateGraph();
    } else if (msg.data.startsWith('resolver_state_changed ')) {
        const c_r = JSON.parse(msg.data.substring('resolver_state_changed '.length));
        node_map[c_r.id].state = c_r.state;
        links.filter(l => l.source.id === c_r.id).forEach(l => l.state = c_r.state);
        updateGraph();
    } else if (msg.data.startsWith('current_resolver ')) {
        const c_r = JSON.parse(msg.data.substring('current_resolver '.length));
        if (current_resolver) current_resolver.current = false;
        const r_node = node_map[c_r.id];
        r_node.current = true;
        current_resolver = r_node;
        updateGraph();
    } else if (msg.data.startsWith('causal_link_added ')) {
        const c_l = JSON.parse(msg.data.substring('causal_link_added '.length));
        links.push({ source: c_l.flaw_id, target: c_l.resolver_id, state: node_map[c_l.flaw_id].state });
        updateGraph();
        update_resolver_cost(node_map[c_l.resolver_id]);
        updateGraph();
    }
};

function updateGraph() {
    const n_group = g.selectAll('g').data(nodes, d => d.id).join(
        enter => {
            const g = enter.append('g').attr('cursor', 'grab');
            g.append('rect').attr('width', 30).attr('x', -15).attr('height', 10).attr('y', -5).attr('rx', d => radius(d)).attr('ry', d => radius(d)).style('fill', d => node_color(d)).style('stroke-dasharray', d => stroke_dasharray(d)).transition().duration(500).style('stroke', d => stroke(d)).style('stroke-width', d => stroke_width(d));
            g.append('text').attr('y', -7).text(d => d.type === 'flaw' ? flaw_label(d) : resolver_label(d));
            g.on('mouseover', (event, d) => tooltip.html(d.type === 'flaw' ? flaw_tooltip(d) : resolver_tooltip(d)).transition().duration(200).style('opacity', .9))
                .on('mousemove', event => tooltip.style('left', (event.pageX) + 'px').style('top', (event.pageY - 28) + 'px'))
                .on('mouseout', event => tooltip.transition().duration(500).style('opacity', 0));
            g.call(d3.drag().on('start', dragstarted).on('drag', dragged).on('end', dragended));
            return g;
        },
        update => {
            update.select('rect').style('fill', d => node_color(d)).style('stroke-dasharray', d => stroke_dasharray(d)).transition().duration(500).style('stroke', d => stroke(d)).style('stroke-width', d => stroke_width(d));
            return update;
        }
    );
    const l_group = g.selectAll('line').data(links).join(
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

function dragstarted(event, d) {
    if (!event.active) simulation.alphaTarget(0.3).restart();
    d.fx = d.x;
    d.fy = d.y;
    d3.select(this).attr('cursor', 'grabbing');
}

function dragged(event, d) {
    d.fx = event.x;
    d.fy = event.y;
    tooltip.style('left', (event.x) + 'px').style('top', (event.y - 28) + 'px');
}

function dragended(event, d) {
    if (!event.active) simulation.alphaTarget(0);
    d.fx = null;
    d.fy = null;
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
    resolver.cost = c_cost == Number.NEGATIVE_INFINITY ? resolver.intrinsic_cost.num / resolver.intrinsic_cost.den : resolver.intrinsic_cost.num / resolver.intrinsic_cost.den + c_cost;
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

function flaw_tooltip(flaw) {
    return '\u03C6' + flaw.label.phi + ', cost: ' + flaw.cost + ', pos: ' + flaw.position.min;
}

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

function resolver_tooltip(resolver) {
    return '\u03C1' + resolver.label.rho + ', cost: ' + resolver.cost;
}