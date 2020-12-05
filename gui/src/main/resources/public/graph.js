const timelines = [];
let horizon;

const nodes = [];
const node_map = new Map();
const links = [];

let current_flaw, current_resolver;

const timelines_svg = d3.select('#timelines').append('svg');
var sv_ok_lg = timelines_svg.append('defs').append('linearGradient').attr('id', 'sv-ok-lg').attr('x1', '0%').attr('x2', '0%').attr('y1', '0%').attr('y2', '100%');
sv_ok_lg.append('stop').attr('offset', '0%').style('stop-color', 'palegreen').style('stop-opacity', 1);
sv_ok_lg.append('stop').attr('offset', '20%').style('stop-color', 'ivory').style('stop-opacity', 1);
sv_ok_lg.append('stop').attr('offset', '100%').style('stop-color', 'palegreen').style('stop-opacity', 1);
var sv_inc_lg = timelines_svg.append('defs').append('linearGradient').attr('id', 'sv-inc-lg').attr('x1', '0%').attr('x2', '0%').attr('y1', '0%').attr('y2', '100%');
sv_inc_lg.append('stop').attr('offset', '0%').style('stop-color', 'lightsalmon').style('stop-opacity', 1);
sv_inc_lg.append('stop').attr('offset', '20%').style('stop-color', 'ivory').style('stop-opacity', 1);
sv_inc_lg.append('stop').attr('offset', '100%').style('stop-color', 'lightsalmon').style('stop-opacity', 1);
var sv_none_lg = timelines_svg.append('defs').append('linearGradient').attr('id', 'sv-none-lg').attr('x1', '0%').attr('x2', '0%').attr('y1', '0%').attr('y2', '100%');
sv_none_lg.append('stop').attr('offset', '0%').style('stop-color', 'lightsalmon').style('stop-opacity', 1);
sv_none_lg.append('stop').attr('offset', '20%').style('stop-color', 'ivory').style('stop-opacity', 1);
sv_none_lg.append('stop').attr('offset', '100%').style('stop-color', 'lightsalmon').style('stop-opacity', 1);

const timelines_g = timelines_svg.append('g');

const timelines_box = timelines_svg.node().getBoundingClientRect();
const timelines_width = timelines_box.width, timelines_height = timelines_box.height;

const timelines_zoom = d3.zoom().on('zoom', event => timelines_g.attr('transform', event.transform));
timelines_svg.call(timelines_zoom);

const timelines_x_scale = d3.scaleLinear().range([0, timelines_width]);
const timelines_y_scale = d3.scaleBand().rangeRound([0, timelines_height]).padding(0.1);

const graph_svg = d3.select('#graph').append('svg');
const graph_g = graph_svg.append('g');

const graph_box = graph_svg.node().getBoundingClientRect();
const graph_width = graph_box.width, graph_height = graph_box.height;

const graph_zoom = d3.zoom().on('zoom', event => graph_g.attr('transform', event.transform));
graph_svg.call(graph_zoom);

graph_svg.append('svg:defs').append('svg:marker')
    .attr('id', 'triangle')
    .attr('viewBox', '0 -5 10 10')
    .attr('refX', 8)
    .attr('refY', 0)
    .attr('markerWidth', 7)
    .attr('markerHeight', 7)
    .attr('orient', 'auto')
    .append('path')
    .attr('d', 'M0,-5L10,0L0,5');

const color_interpolator = d3.scaleSequential(d3.interpolateRdYlGn).domain([15, 0]);

const tooltip = d3.select('body').append('div').attr('class', 'tooltip').style('opacity', 0);

const simulation = d3.forceSimulation(nodes)
    .force('link', d3.forceLink().id(d => d.id).distance(70))
    .force('charge', d3.forceManyBody().strength(-70))
    .force('center', d3.forceCenter(graph_width / 2, graph_height / 2));

updateGraph();

const ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/graph');
ws.onmessage = msg => {
    if (msg.data.startsWith('graph ')) {
        nodes.length = 0;
        links.length = 0;
        node_map.clear();

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
            node_map.set(c_f.id, c_f);
            c_f.causes.forEach(c => links.push({ source: c_f.id, target: c, state: c_f.state }));
        });
        c_graph.resolvers.forEach(r => {
            const c_r = JSON.parse(r);
            c_r.label = JSON.parse(c_r.label);
            c_r.cost = c_r.intrinsic_cost.num / c_r.intrinsic_cost.den;
            c_r.type = 'resolver';

            nodes.push(c_r);
            node_map.set(c_r.id, c_r);
            links.push({ source: c_r.id, target: c_r.effect, state: c_r.state });
            c_r.preconditions.filter(pre => !links.find(l => l.source === pre && l.target === c_r.id)).forEach(pre => links.push({ source: pre, target: c_r.id, state: node_map.get(pre).state }));
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
        node_map.set(c_f.id, c_f);
        c_f.causes.forEach(c => links.push({ source: c_f.id, target: c, state: c_f.state }));
        updateGraph();
        c_f.causes.forEach(c => update_resolver_cost(node_map.get(c)));
        updateGraph();
    } else if (msg.data.startsWith('flaw_state_changed ')) {
        const c_f = JSON.parse(msg.data.substring('flaw_state_changed '.length));
        node_map.get(c_f.id).state = c_f.state;
        links.filter(l => l.source.id === c_f.id).forEach(l => l.state = c_f.state);
        updateGraph();
    } else if (msg.data.startsWith('flaw_cost_changed ')) {
        const c_f = JSON.parse(msg.data.substring('flaw_cost_changed '.length));
        const f_node = node_map.get(c_f.id);
        f_node.cost = c_f.cost.num / c_f.cost.den;
        links.filter(l => l.source.id == f_node.id).forEach(out_link => update_resolver_cost(out_link.target));
        updateGraph();
    } else if (msg.data.startsWith('flaw_position_changed ')) {
        const c_f = JSON.parse(msg.data.substring('flaw_position_changed '.length));
        node_map.get(c_f.id).position = c_f.position;
        updateGraph();
    } else if (msg.data.startsWith('current_flaw ')) {
        const c_f = JSON.parse(msg.data.substring('current_flaw '.length));
        if (current_flaw) current_flaw.current = false;
        if (current_resolver) { current_resolver.current = false; current_resolver = undefined; }
        const f_node = node_map.get(c_f.id);
        f_node.current = true;
        current_flaw = f_node;
        updateGraph();
    } else if (msg.data.startsWith('resolver_created ')) {
        const c_r = JSON.parse(msg.data.substring('resolver_created '.length));
        c_r.label = JSON.parse(c_r.label);
        c_r.cost = c_r.intrinsic_cost.num / c_r.intrinsic_cost.den;
        c_r.type = 'resolver';

        nodes.push(c_r);
        node_map.set(c_r.id, c_r);
        links.push({ source: c_r.id, target: c_r.effect, state: c_r.state });
        updateGraph();
    } else if (msg.data.startsWith('resolver_state_changed ')) {
        const c_r = JSON.parse(msg.data.substring('resolver_state_changed '.length));
        node_map.get(c_r.id).state = c_r.state;
        links.filter(l => l.source.id === c_r.id).forEach(l => l.state = c_r.state);
        updateGraph();
    } else if (msg.data.startsWith('current_resolver ')) {
        const c_r = JSON.parse(msg.data.substring('current_resolver '.length));
        if (current_resolver) current_resolver.current = false;
        const r_node = node_map.get(c_r.id);
        r_node.current = true;
        current_resolver = r_node;
        updateGraph();
    } else if (msg.data.startsWith('causal_link_added ')) {
        const c_l = JSON.parse(msg.data.substring('causal_link_added '.length));
        links.push({ source: c_l.flaw_id, target: c_l.resolver_id, state: node_map.get(c_l.flaw_id).state });
        updateGraph();
        update_resolver_cost(node_map.get(c_l.resolver_id));
        updateGraph();
    } else if (msg.data.startsWith('timelines ')) {
        const ts = JSON.parse(msg.data.substring('timelines '.length));
        timelines.splice(0, timelines.length - ts.length);
        ts.forEach((tl, i) => {
            timelines[i] = tl;
            timelines[i].id = i;
            timelines[i].values.forEach((v, j) => v.id = j);
            if (timelines[i].type === 'reusable-resource' && timelines[i].values.length) timelines[i].values.push({ usage: 0, from: timelines[i].values[timelines[i].values.length - 1].to, id: timelines[i].values.length });
        });
        horizon = Math.max(d3.max(timelines, d => d.horizon), 1);
        timelines_x_scale.domain([0, horizon]);
        timelines_y_scale.domain(d3.range(timelines.length));
        updateTimelines();
    }
};

function updateTimelines() {
    timelines_g.selectAll('g').data(timelines, d => d.id).join(
        enter => {
            const tl_g = enter.append('g').attr('class', 'timeline').attr('id', d => 'tl-' + d.id);
            tl_g.append('rect').attr('x', -10).attr('y', d => timelines_y_scale(timelines.indexOf(d))).attr('width', timelines_x_scale(horizon) + 20).attr('height', timelines_y_scale.bandwidth()).style('fill', 'floralwhite');
            tl_g.append('text').attr('x', 0).attr('y', d => timelines_y_scale(timelines.indexOf(d)) + timelines_y_scale.bandwidth() * 0.08).text(d => d.name).style('text-anchor', 'start');
            return tl_g;
        },
        update => {
            update.select('rect').transition().duration(200).attr('width', timelines_x_scale(horizon) + 20);
            return update;
        }
    );
    timelines.forEach((tl, i) => updateTimeline(tl, i));
}

function updateTimeline(tl, i) {
    switch (tl.type) {
        case 'state-variable':
            d3.select('#tl-' + i).selectAll('g').data(tl.values, d => d.id).join(
                enter => {
                    const tl_val_g = enter.append('g');
                    tl_val_g.append('rect').attr('x', d => timelines_x_scale(d.from)).attr('y', d => timelines_y_scale(i) + timelines_y_scale.bandwidth() * 0.1).attr('width', d => timelines_x_scale(d.to) - timelines_x_scale(d.from)).attr('height', timelines_y_scale.bandwidth() * 0.9).attr('rx', 5).attr('ry', 5).style('fill', d => sv_value_fill(d)).style('stroke', 'lightgray');
                    tl_val_g.on('mouseover', (event, d) => tooltip.html(sv_value_tooltip(d)).transition().duration(200).style('opacity', .9))
                        .on('mousemove', event => tooltip.style('left', (event.pageX) + 'px').style('top', (event.pageY - 28) + 'px'))
                        .on('mouseout', event => tooltip.transition().duration(500).style('opacity', 0));
                    return tl_val_g;
                },
                update => {
                    update.select('rect').transition().duration(200).attr('x', d => timelines_x_scale(d.from)).attr('y', d => timelines_y_scale(i) + timelines_y_scale.bandwidth() * 0.1).attr('width', d => timelines_x_scale(d.to) - timelines_x_scale(d.from)).attr('height', timelines_y_scale.bandwidth() * 0.9).style('fill', d => sv_value_fill(d));
                    return update;
                }
            );
            break;
        case 'reusable-resource':
            const rr_max_val = (tl.values.length ? Math.max(d3.max(tl.values, d => d.usage), tl.capacity) : tl.capacity);
            const rr_y_scale = d3.scaleLinear().domain([0, rr_max_val + rr_max_val * 0.1]).range([timelines_y_scale(i) + timelines_y_scale.bandwidth(), timelines_y_scale(i)]);
            const rr_g = d3.select('#tl-' + i);
            rr_g.selectAll('path').data([tl.values]).join(
                enter => {
                    const tl_val_g = enter.append('path').attr('fill', 'aliceblue').attr('stroke', 'lightblue')
                        .attr('d', d3.area().curve(d3.curveStepAfter).x(d => timelines_x_scale(d.from)).y0(timelines_y_scale(i) + timelines_y_scale.bandwidth()).y1(d => rr_y_scale(d.usage)));
                    tl_val_g.on('mouseover', (event, d) => tooltip.html(rr_value_tooltip(d)).transition().duration(200).style('opacity', .9))
                        .on('mousemove', event => tooltip.style('left', (event.pageX) + 'px').style('top', (event.pageY - 28) + 'px'))
                        .on('mouseout', event => tooltip.transition().duration(500).style('opacity', 0));
                    return tl_val_g;
                },
                update => {
                    update.transition().duration(200).attr('d', d3.area().curve(d3.curveStepAfter).x(d => timelines_x_scale(d.from)).y0(timelines_y_scale(i) + timelines_y_scale.bandwidth()).y1(d => rr_y_scale(d.usage)));
                    return update;
                }
            );
            rr_g.selectAll('line').data([tl.capacity]).join(
                enter => {
                    const line_g = enter.append('line').attr('stroke-width', 2).attr('stroke-linecap', 'round').attr('stroke', 'darkslategray');
                    line_g.attr('x1', timelines_x_scale(0)).attr('y1', d => rr_y_scale(d)).attr('x2', timelines_x_scale(horizon)).attr('y2', d => rr_y_scale(d));
                    return line_g;
                },
                update => {
                    update.transition().duration(200).attr('y1', d => rr_y_scale(d)).attr('y2', d => rr_y_scale(d));
                    return update;
                }
            );
            break;
        default:
            break;
    }
}

function updateGraph() {
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

function sv_value_tooltip(sv_value) { return sv_value.name; }

function rr_value_tooltip(rr_value) { return rr_value.usage; }

function sv_value_fill(sv_value) {
    switch (sv_value.atoms.length) {
        case 0: return 'url(#sv-none-lg)';
        case 1: return 'url(#sv-ok-lg)';
        default: return 'url(#sv-inc-lg)';
    }
}