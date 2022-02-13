const sc = chroma.scale(['#90EE90', 'yellow', 'red']).domain([0, 10]);

let core_ws, solver_ws, executor_ws;

setup_ws();

const nodes = new vis.DataSet([]);
const edges = new vis.DataSet([]);

const data = {
    nodes: nodes,
    edges: edges
};
const options = {};

const network = new vis.Network(document.getElementById('graph'), data, options);

function setup_ws() {
    solver_ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/solver');
    solver_ws.onmessage = msg => {
        const c_msg = JSON.parse(msg.data);
        switch (c_msg.type) {
            case 'StartedSolving':
                console.log('solving the problem..');
                break;
            case 'SolutionFound':
                console.log('hurray!! we have found a solution..');
                break;
            case 'InconsistentProblem':
                console.log('the problem has no solution..');
                break;
        }
    };
    solver_ws.onclose = () => setTimeout(setup_ws, 1000);
}