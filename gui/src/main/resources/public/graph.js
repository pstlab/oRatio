const flaws = [];
const resolvers = [];

const nodes = [];
const links = [];

const svg = d3.select('#graph').append('svg');
const g = svg.append('g');

const b_box = svg.node().getBoundingClientRect();
const width = b_box.width, height = b_box.height;

const c_zoom = d3.zoom().on('zoom', event => g.attr('transform', event.transform));
svg.call(c_zoom);

const simulation = d3.forceSimulation(nodes)
    .force('link', d3.forceLink().id(d => d.id))
    .force('charge', d3.forceManyBody())
    .force('center', d3.forceCenter(width / 2, height / 2));

const ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/graph');
ws.onmessage = msg => {
    if (msg.data.startsWith('graph ')) {
        flaws.length = 0;
        resolvers.length = 0;
        nodes.length = 0;
        links.length = 0;

        const c_graph = JSON.parse(msg.data.substring(6));
        c_graph.flaws.forEach(f => {
            const c_f = JSON.parse(f);
            c_f.label = JSON.parse(c_f.label);
            flaws.push(c_f);

            nodes.push({ id: c_f.id, label: flaw_label(c_f.label), cost: c_f.cost.num / c_f.cost.den, type: 'flaw' });
            c_f.causes.forEach(c => links.push({ source: c_f.id, target: c.id, state: c_f.state }));
        });
        c_graph.resolvers.forEach(r => {
            const c_r = JSON.parse(r);
            c_r.label = JSON.parse(c_r.label);
            resolvers.push(c_r);

            nodes.push({ id: c_r.id, label: resolver_label(c_r.label), cost: c_r.cost.num / c_r.cost.den, type: 'resolver' });
            links.push({ source: c_r.id, target: c_r.effect, state: c_r.state });
        });
    }
    updateGraph();
};

updateGraph();

function updateGraph() {
    const n_group = g.selectAll('g').data(nodes, d => d.id).join(
        enter => {
            const g = enter.append('g').attr('cursor', 'grab');
            g.append('rect').attr('width', 30).attr('x', -15).attr('height', 10).attr('y', -5).attr('rx', 5).attr('ry', 5).style('fill', 'pink');
            g.append('text').text(d => d.label);
            g.call(d3.drag().on('start', dragstarted).on('drag', dragged).on('end', dragended));
            return g;
        }
    );
    const l_group = g.selectAll('line').data(links).join(enter => enter.append('line').attr('stroke', 'black'));

    simulation.nodes(nodes).on('tick', () => {
        n_group.attr('transform', d => `translate(${d.x}, ${d.y})`);
        l_group.attr('x1', d => d.source.x).attr('y1', d => d.source.y).attr('x2', d => d.target.x).attr('y2', d => d.target.y);
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

function flaw_label(label) {
    switch (label.type) {
        case 'enum':
            return 'ϕ' + label.phi + ' enum';
        case 'bool':
            return 'ϕ' + label.phi + ' bool';
        default:
            return 'ϕ' + label.phi + ' ' + label.type;
    }
}

function resolver_label(label) {
    if (label.type)
        switch (label.type) {
            case 'activate':
                return 'ρ' + label.rho + ' activate';
            case 'unify':
                return 'ρ' + label.rho + ' unify';
            default:
                return 'ρ' + label.rho + ' ' + label.type;
        }
    return 'ρ' + label.rho;
}