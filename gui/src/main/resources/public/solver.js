import { Graph, GraphData } from './modules/graph.js';
import { Timelines, TimelinesData } from './modules/timelines.js';

document.querySelector('#tick').addEventListener('click', () => ws.send('tick'));

const timelines_data = new TimelinesData();
const timelines_chart = new Timelines();
const graph_data = new GraphData();
const graph = new Graph();

setup_ws();

function setup_ws() {
    const ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/solver');
    ws.onmessage = msg => {
        const c_msg = JSON.parse(msg.data);
        switch (c_msg.message_type) {
            case 'graph':
                c_msg.flaws.forEach(f => {
                    f.label = JSON.parse(f.label);
                    if (f.cost)
                        f.cost = (f.cost.num / f.cost.den);
                    else
                        f.cost = Number.POSITIVE_INFINITY;
                });
                c_msg.resolvers.forEach(r => {
                    r.label = JSON.parse(r.label);
                    r.intrinsic_cost = r.cost.num / r.cost.den;
                    r.cost = r.intrinsic_cost;
                });
                graph_data.reset(c_msg.flaws, c_msg.resolvers);
                graph.update(graph_data);
                break;
            case 'started_solving':
                console.log('solving the problem..');
                break;
            case 'solution_found':
                console.log('hurray!! we have found a solution..');
                graph_data.solution_found();
                graph.update(graph_data);
                break;
            case 'inconsistent_problem':
                console.log('unsolvable problem..');
                break;
            case 'flaw_created':
                c_msg.label = JSON.parse(c_msg.label);
                if (c_msg.cost)
                    c_msg.cost = (c_msg.cost.num / c_msg.cost.den);
                else
                    c_msg.cost = Number.POSITIVE_INFINITY;
                graph_data.flaw_created(c_msg);
                graph.update(graph_data);
                break;
            case 'flaw_state_changed':
                graph_data.flaw_state_changed(c_msg);
                graph.update(graph_data);
                break;
            case 'flaw_cost_changed':
                c_msg.cost = c_msg.cost.num / c_msg.cost.den;
                graph_data.flaw_cost_changed(c_msg);
                graph.update(graph_data);
                break;
            case 'flaw_position_changed':
                graph_data.flaw_position_changed(c_msg);
                graph.update(graph_data);
                break;
            case 'current_flaw':
                graph_data.current_flaw(c_msg);
                graph.update(graph_data);
                break;
            case 'resolver_created':
                c_msg.label = JSON.parse(c_msg.label);
                c_msg.intrinsic_cost = c_msg.cost.num / c_msg.cost.den;
                c_msg.cost = c_msg.intrinsic_cost;
                graph_data.resolver_created(c_msg);
                graph.update(graph_data);
                break;
            case 'resolver_state_changed':
                graph_data.resolver_state_changed(c_msg);
                graph.update(graph_data);
                break;
            case 'current_resolver':
                graph_data.current_resolver(c_msg);
                graph.update(graph_data);
                break;
            case 'causal_link_added':
                graph_data.causal_link_added(c_msg);
                graph.update(graph_data);
                break;
            case 'timelines':
                timelines_data.reset(c_msg.timelines);
                timelines_chart.update(timelines_data);
                break;
            case 'tick':
                timelines_data.tick(c_msg.current_time.num / c_msg.current_time.den);
                timelines_chart.updateTime(timelines_data);
                break;
            case 'starting_atoms':
                timelines_data.starting_atoms(c_msg);
                break;
            case 'ending_atoms':
                timelines_data.ending_atoms(c_msg);
                break;
            default:
                console.error('cannot handle message type ' + c_msg.message_type);
                break;
        }
    };
    ws.onclose = () => setTimeout(setup_ws, 1000);
}