const flaws = [];
const resolvers = [];

const nodes = [];
const links = [];

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

var color_interpolator = d3.scaleSequential().domain([1, 10]).interpolator(d3.interpolateRdYlGn);

const simulation = d3.forceSimulation(nodes)
    .force('link', d3.forceLink().id(d => d.id).distance(60))
    .force('charge', d3.forceManyBody().strength(-60))
    .force('center', d3.forceCenter(width / 2, height / 2));

const ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/graph');
ws.onmessage = msg => {
    if (msg.data.startsWith('graph ')) {
        flaws.length = 0;
        resolvers.length = 0;
        nodes.length = 0;
        links.length = 0;

        const c_graph = JSON.parse(msg.data.substring('graph '.length));
        c_graph.flaws.forEach(f => {
            const c_f = JSON.parse(f);
            c_f.label = JSON.parse(c_f.label);
            if (!c_f.cost) c_f.cost = Number.POSITIVE_INFINITY;
            flaws.push(c_f);

            nodes.push({ id: c_f.id, label: flaw_label(c_f.label), cost: c_f.cost.num / c_f.cost.den, state: c_f.state, type: 'flaw' });
            c_f.causes.forEach(c => links.push({ source: c_f.id, target: c, state: c_f.state }));
        });
        c_graph.resolvers.forEach(r => {
            const c_r = JSON.parse(r);
            c_r.label = JSON.parse(c_r.label);
            resolvers.push(c_r);

            nodes.push({ id: c_r.id, label: resolver_label(c_r.label), intrinsic_cost: c_r.cost.num / c_r.cost.den, cost: c_r.cost.num / c_r.cost.den, state: c_r.state, type: 'resolver' });
            links.push({ source: c_r.id, target: c_r.effect, state: c_r.state });
        });
    } else if (msg.data.startsWith('flaw_created ')) {
        const c_f = JSON.parse(msg.data.substring('flaw_created '.length));
        c_f.label = JSON.parse(c_f.label);
        if (!c_f.cost) c_f.cost = Number.POSITIVE_INFINITY;
        flaws.push(c_f);

        nodes.push({ id: c_f.id, label: flaw_label(c_f.label), cost: c_f.cost.num / c_f.cost.den, state: c_f.state, type: 'flaw' });
        c_f.causes.forEach(c => links.push({ source: c_f.id, target: c, state: c_f.state }));
    } else if (msg.data.startsWith('flaw_state_changed ')) {
        const c_f = JSON.parse(msg.data.substring('flaw_state_changed '.length));
        getNode(c_f.id).state = c_f.state;
    } else if (msg.data.startsWith('flaw_cost_changed ')) {
        const c_f = JSON.parse(msg.data.substring('flaw_cost_changed '.length));
        const node = getNode(c_f.id);
        node.cost = c_f.cost.num / c_f.cost.den;
        links.filter(l => l.source == node.id).forEach(out_link => {
            const c_r = getNode(out_link.target);
            let c_cost = Number.POSITIVE_INFINITY;
            links.filter(l => l.target == out_link.target).forEach(in_link => {
                const c_in_f = getNode(in_link.source); // a sub-flaw..
                if (c_cost < c_in_f.cost)
                    c_cost = c_in_f.cost;
            });
            c_r.cost = c_cost;
        });
    } else if (msg.data.startsWith('resolver_created ')) {
        const c_r = JSON.parse(msg.data.substring('resolver_created '.length));
        c_r.label = JSON.parse(c_r.label);
        resolvers.push(c_r);

        nodes.push({ id: c_r.id, label: resolver_label(c_r.label), intrinsic_cost: c_r.cost.num / c_r.cost.den, cost: c_r.cost.num / c_r.cost.den, state: c_r.state, type: 'resolver' });
        links.push({ source: c_r.id, target: c_r.effect, state: c_r.state });
    } else if (msg.data.startsWith('resolver_state_changed ')) {
        const c_r = JSON.parse(msg.data.substring('resolver_state_changed '.length));
        getNode(c_r.id).state = c_r.state;
    }
    updateGraph();
};

updateGraph();

function updateGraph() {
    const n_group = g.selectAll('g').data(nodes, d => d.id).join(
        enter => {
            const g = enter.append('g').attr('cursor', 'grab');
            g.append('rect').attr('width', 30).attr('x', -15).attr('height', 10).attr('y', -5).attr('rx', d => radius(d)).attr('ry', d => radius(d)).style('fill', d => node_color(d)).style('stroke', 'black').style('stroke-dasharray', d => stroke_dasharray(d));
            g.append('text').attr('y', -7).text(d => d.label);
            g.call(d3.drag().on('start', dragstarted).on('drag', dragged).on('end', dragended));
            return g;
        }
    );
    const l_group = g.selectAll('line').data(links).join(enter => enter.append('line').attr('stroke', 'black').style('stroke-dasharray', d => stroke_dasharray(d)));

    simulation.nodes(nodes).on('tick', () => {
        n_group.attr('transform', d => `translate(${d.x}, ${d.y})`);
        l_group.attr('x1', d => d.source.x).attr('y1', d => d.source.y).attr('x2', d => d.target.x).attr('y2', d => d.target.y).attr('marker-end', 'url(#triangle)');
    });
    simulation.force('link').links(links);

    simulation.restart();
    simulation.alpha(0.3);
}

function getNode(id) { return nodes.find(x => x.id === id); }

function addNode(n) {
    nodes.push(n);
    updateGraph();
    simulation.restart();
    simulation.alpha(0.3);
}

function addNodes(ns) {
    ns.forEach(n => nodes.push(n));
    updateGraph();
    simulation.restart();
    simulation.alpha(0.3);
}

function addLink(n0, n1) {
    links.push({ source: n0.id, target: n1.id, type: 'c' });
    updateGraph();
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
}

function dragended(event, d) {
    if (!event.active) simulation.alphaTarget(0);
    d.fx = null;
    d.fy = null;
    d3.select(this).attr('cursor', 'grab');
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

function stroke_dasharray(n) {
    switch (n.state) {
        case 0: // False
            return '1';
        case 1: // True
            return '100';
        case 2: // Undefined
            return '2';
        default:
            break;
    }
}

function node_color(n) {
    switch (n.state) {
        case 0: // False
            return '#d9d9d9';
        default:
            return color_interpolator(n.cost);
    }
}

function flaw_label(label) {
    switch (label.type) {
        case 'enum':
            return '\u03C6' + label.phi + ' enum';
        case 'bool':
            return '\u03C6' + label.phi + ' bool';
        default:
            return '\u03C6' + label.phi + ' ' + label.type;
    }
}

function resolver_label(label) {
    if (label.type)
        switch (label.type) {
            case 'activate':
                return '\u03C1' + label.rho + ' activate';
            case 'unify':
                return '\u03C1' + label.rho + ' unify';
            default:
                return '\u03C1' + label.rho + ' ' + label.type;
        }
    return '\u03C1' + label.rho;
}