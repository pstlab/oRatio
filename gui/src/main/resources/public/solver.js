import { Graph, GraphData } from './modules/graph.js';
import { Timelines, TimelinesData } from './modules/timelines.js';

let ws;
document.querySelector('#tick').addEventListener('click', () => ws.send('tick'));

const timelines_data = new TimelinesData();
const timelines_chart = new Timelines();
const graph_data = new GraphData();
const graph = new Graph();

setup_ws();

function setup_ws() {
    ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/solver');
    ws.onmessage = msg => {
        const c_msg = JSON.parse(msg.data);
        switch (c_msg.type) {
            case 'Graph':
                graph_data.reset(c_msg);
                graph.update(graph_data);
                break;
            case 'StartedSolving':
                console.log('solving the problem..');
                break;
            case 'SolutionFound':
                console.log('hurray!! we have found a solution..');
                graph_data.solution_found();
                graph.update(graph_data);
                break;
            case 'InconsistentProblem':
                console.log('unsolvable problem..');
                break;
            case 'FlawCreated':
                graph_data.flaw_created(c_msg);
                graph.update(graph_data);
                break;
            case 'FlawStateChanged':
                graph_data.flaw_state_changed(c_msg);
                graph.update(graph_data);
                break;
            case 'FlawCostChanged':
                graph_data.flaw_cost_changed(c_msg);
                graph.update(graph_data);
                break;
            case 'FlawPositionChanged':
                graph_data.flaw_position_changed(c_msg);
                graph.update(graph_data);
                break;
            case 'CurrentFlaw':
                graph_data.current_flaw(c_msg);
                graph.update(graph_data);
                break;
            case 'ResolverCreated':
                graph_data.resolver_created(c_msg);
                graph.update(graph_data);
                break;
            case 'ResolverStateChanged':
                graph_data.resolver_state_changed(c_msg);
                graph.update(graph_data);
                break;
            case 'CurrentResolver':
                graph_data.current_resolver(c_msg);
                graph.update(graph_data);
                break;
            case 'CausalLinkAdded':
                graph_data.causal_link_added(c_msg);
                graph.update(graph_data);
                break;
            case 'Timelines':
                timelines_data.reset(c_msg);
                timelines_chart.update(timelines_data);
                break;
            case 'Tick':
                timelines_data.tick(c_msg);
                timelines_chart.updateTime(timelines_data);
                break;
            case 'StartingAtoms':
                timelines_data.starting_atoms(c_msg);
                break;
            case 'EndingAtoms':
                timelines_data.ending_atoms(c_msg);
                break;
            default:
                console.error('cannot handle message type ' + c_msg.type);
                break;
        }
    };
    ws.onclose = () => setTimeout(setup_ws, 1000);
}