const nodes = [{ id: 0, label: 'a' }, { id: 1, label: 'b' }, { id: 2, label: 'c' }];
const links = [{ source: 0, target: 1, type: 'a' }, { source: 1, target: 2, type: 'b' }];

const svg = d3.select("svg");
const g = svg.append("g");

const b_box = svg.node().getBoundingClientRect();
const width = b_box.width, height = b_box.height;

let c_zoom = d3.zoom().on('zoom', event => g.attr('transform', event.transform));
svg.call(c_zoom);

const simulation = d3.forceSimulation(nodes)
    .force('link', d3.forceLink().id(d => d.id))
    .force('charge', d3.forceManyBody())
    .force('center', d3.forceCenter(width / 2, height / 2));

function update() {
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
}

update();

function getNode(id) { return nodes[id]; }

function addNode(n) {
    nodes.push(n);
    update();
    simulation.restart();
    simulation.alpha(0.3);
}

function addLink(n0, n1) {
    links.push({ source: n0.id, target: n1.id, type: 'c' });
    update();
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