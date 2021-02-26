import { Graph } from "./modules/graph.js";
import { Timelines } from "./modules/timelines.js";

const tooltip = d3.select('body').append('div').attr('class', 'tooltip').style('opacity', 0);

const timelines = new Timelines(d3.select('#timelines').append('svg'), tooltip);
const graph = new Graph(d3.select('#graph').append('svg'), tooltip);

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
                r.cost = r.cost.num / r.cost.den;
            });
            graph.reset(c_msg);
            break;
        case 'flaw_created':
            c_msg.label = JSON.parse(c_msg.label);
            if (c_msg.cost)
                c_msg.cost = (c_msg.cost.num / c_msg.cost.den);
            else
                c_msg.cost = Number.POSITIVE_INFINITY;
            graph.flaw_created(c_msg);
            break;
        case 'flaw_state_changed':
            graph.flaw_state_changed(c_msg);
            break;
        case 'flaw_cost_changed':
            c_msg.cost = c_msg.cost.num / c_msg.cost.den;
            graph.flaw_cost_changed(c_msg);
            break;
        case 'flaw_position_changed':
            graph.flaw_position_changed(c_msg);
            break;
        case 'current_flaw': {
            graph.current_flaw(c_msg);
            break;
        }
        case 'resolver_created':
            c_msg.label = JSON.parse(c_msg.label);
            c_msg.intrinsic_cost = c_msg.cost.num / c_msg.cost.den;
            c_msg.cost = c_msg.intrinsic_cost;
            graph.resolver_created(c_msg);
            break;
        case 'resolver_state_changed':
            graph.resolver_state_changed(c_msg);
            break;
        case 'current_resolver':
            graph.current_resolver(c_msg);
            break;
        case 'causal_link_added':
            graph.causal_link_added(c_msg);
            break;
        case 'timelines':
            timelines.reset(c_msg);
            break;
        case 'tick':
            timelines.tick(c_msg.current_time.num / c_msg.current_time.den);
            break;
        case 'starting_atoms':
            timelines.starting_atoms(c_msg);
            break;
        case 'ending_atoms':
            timelines.ending_atoms(c_msg);
            break;
        default:
            console.error('cannot handle message type ' + c_msg.message_type);
            break;
    }
};
