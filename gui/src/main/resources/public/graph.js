const nodes = [{ id: 0, label: 'a' }, { id: 1, label: 'b' }, { id: 2, label: 'c' }];
const links = [{ source: 0, target: 1, type: 'a' }, { source: 1, target: 2, type: 'b' }];

const vis = d3.select('#vis');
const width = vis.attr('width'), height = vis.attr('height');

let zoom = d3.zoom().on("zoom", (event) => {
    vis.attr("transform",
        `translate(${event.transform.x}, ${event.transform.y})` + " " +
        `scale(${event.transform.k})`);
});
vis.call(zoom);

const simulation = d3.forceSimulation(nodes)
    .force('link', d3.forceLink().id(d => d.id))
    .force('charge', d3.forceManyBody())
    .force('center', d3.forceCenter(width / 2, height / 2));

var rainbow = d3.scaleSequential(d3.interpolateRainbow);

function update() {
    const n_group = vis.selectAll('g').data(nodes, d => d.id);
    const entering_nodes = n_group.enter().append('g').call(d3.drag().on('start', dragstarted).on('drag', dragged).on('end', dragended));
    entering_nodes.append('rect').attr('width', 30).attr('x', -15).attr('height', 10).attr('y', -5).attr('rx', 5).attr('ry', 5).style('fill', 'pink');
    entering_nodes.append('text').text(d => d.label);

    const l_group = vis.selectAll('line').data(links);
    const entering_links = l_group.enter().append('line').attr('stroke', 'black');

    simulation.nodes(nodes).on('tick', () => {
        l_group.attr('x1', d => d.source.x).attr('y1', d => d.source.y).attr('x2', d => d.target.x).attr('y2', d => d.target.y);
        n_group.attr('transform', d => `translate(${d.x}, ${d.y})`);
    });
    simulation.force("link").links(links);
}

update();

function getNode(id) { return nodes[id]; }

function addNode(n) {
    nodes.push(n);
    update();
    simulation.restart();
    simulation.alpha(1);
}

function addLink(n0, n1) {
    links.push({ source: n0.id, target: n1.id, type: 'c' });
    update();
    simulation.restart();
    simulation.alpha(1);
}

function dragstarted(event, d) {
    if (!event.active) simulation.alphaTarget(0.3).restart();
    d.fx = d.x;
    d.fy = d.y;
}

function dragged(event, d) {
    d.fx = event.x;
    d.fy = event.y;
}

function dragended(event, d) {
    if (!event.active) simulation.alphaTarget(0);
    d.fx = null;
    d.fy = null;
}